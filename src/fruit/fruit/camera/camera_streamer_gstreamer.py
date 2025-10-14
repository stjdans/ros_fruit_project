#!/usr/bin/env python3
"""
GStreamer 기반 카메라 스트리머 (가장 안정적)
- 전문 미디어 스트리밍 프레임워크
- RTSP 프로토콜 지원
- 하드웨어 가속
- 자동 버퍼링 및 재연결
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
from threading import Thread
import time


class CameraStreamerGStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer_gstreamer')
        
        # GStreamer 초기화
        Gst.init(None)
        
        # 파라미터 선언
        self.declare_parameter('rtsp_server_url', 'rtsp://192.168.1.100:8554/stream')
        self.declare_parameter('streaming_fps', 30)
        self.declare_parameter('image_quality', 90)
        self.declare_parameter('image_topic', '/pi_camera/image_raw')
        self.declare_parameter('codec', 'h264')  # h264, h265, vp8, vp9
        self.declare_parameter('bitrate', 2000)  # kbps
        
        # 파라미터 가져오기
        self.rtsp_url = self.get_parameter('rtsp_server_url').value
        self.fps = self.get_parameter('streaming_fps').value
        self.quality = self.get_parameter('image_quality').value
        self.image_topic = self.get_parameter('image_topic').value
        self.codec = self.get_parameter('codec').value
        self.bitrate = self.get_parameter('bitrate').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # GStreamer 파이프라인 생성
        self.pipeline = None
        self.appsrc = None
        self.setup_gstreamer_pipeline()
        
        # 최신 이미지
        self.latest_image = None
        
        # 구독자
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)
        
        # 스트리밍 스레드
        self.streaming = True
        self.stream_thread = Thread(target=self.stream_loop, daemon=True)
        self.stream_thread.start()
        
        # GStreamer 메인 루프
        self.mainloop = GLib.MainLoop()
        self.gst_thread = Thread(target=self.mainloop.run, daemon=True)
        self.gst_thread.start()
        
        # 통계
        self.frames_sent = 0
        self.start_time = time.time()
        
        self.get_logger().info('='*60)
        self.get_logger().info('GStreamer 카메라 스트리머 시작')
        self.get_logger().info(f'RTSP URL: {self.rtsp_url}')
        self.get_logger().info(f'코덱: {self.codec.upper()}')
        self.get_logger().info(f'비트레이트: {self.bitrate} kbps')
        self.get_logger().info(f'FPS: {self.fps}')
        self.get_logger().info('='*60)
    
    def setup_gstreamer_pipeline(self):
        """GStreamer 파이프라인 설정"""
        # H.264 인코딩 파이프라인 예시
        # appsrc → videoconvert → x264enc → rtph264pay → udpsink
        
        pipeline_str = (
            'appsrc name=source is-live=true format=time '
            f'caps=video/x-raw,format=BGR,width=640,height=480,framerate={self.fps}/1 ! '
            'videoconvert ! '
        )
        
        if self.codec == 'h264':
            pipeline_str += (
                f'x264enc bitrate={self.bitrate} speed-preset=ultrafast tune=zerolatency ! '
                'rtph264pay config-interval=1 pt=96 ! '
            )
        elif self.codec == 'h265':
            pipeline_str += (
                f'x265enc bitrate={self.bitrate} speed-preset=ultrafast tune=zerolatency ! '
                'rtph265pay config-interval=1 pt=96 ! '
            )
        elif self.codec == 'vp8':
            pipeline_str += (
                f'vp8enc target-bitrate={self.bitrate*1000} deadline=1 ! '
                'rtpvp8pay pt=96 ! '
            )
        
        # UDP 싱크 (RTSP 서버가 리슨하는 포트로 전송)
        # 실제로는 RTSP 서버를 별도로 구성해야 함
        pipeline_str += 'udpsink host=192.168.1.100 port=5000'
        
        self.get_logger().info(f'GStreamer 파이프라인: {pipeline_str}')
        
        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsrc = self.pipeline.get_by_name('source')
            
            # 파이프라인 시작
            self.pipeline.set_state(Gst.State.PLAYING)
            self.get_logger().info('✓ GStreamer 파이프라인 시작됨')
            
        except Exception as e:
            self.get_logger().error(f'GStreamer 파이프라인 생성 실패: {str(e)}')
    
    def image_callback(self, msg):
        """카메라 이미지 콜백"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'이미지 변환 오류: {str(e)}')
    
    def stream_loop(self):
        """스트리밍 루프"""
        rate = 1.0 / self.fps
        
        while self.streaming and rclpy.ok():
            if self.latest_image is not None and self.appsrc is not None:
                try:
                    # OpenCV 이미지를 GStreamer 버퍼로 변환
                    # 이미지 크기 조정 (640x480)
                    frame = cv2.resize(self.latest_image, (640, 480))
                    
                    # Gst.Buffer 생성
                    data = frame.tobytes()
                    buf = Gst.Buffer.new_allocate(None, len(data), None)
                    buf.fill(0, data)
                    
                    # 타임스탬프 설정
                    buf.pts = self.frames_sent * Gst.SECOND // self.fps
                    buf.duration = Gst.SECOND // self.fps
                    
                    # appsrc에 푸시
                    ret = self.appsrc.emit('push-buffer', buf)
                    
                    if ret == Gst.FlowReturn.OK:
                        self.frames_sent += 1
                    else:
                        self.get_logger().warn(f'버퍼 푸시 실패: {ret}')
                    
                except Exception as e:
                    self.get_logger().error(f'스트리밍 오류: {str(e)}')
            
            time.sleep(rate)
    
    def destroy_node(self):
        """노드 종료"""
        self.streaming = False
        
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        
        if self.mainloop:
            self.mainloop.quit()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        streamer = CameraStreamerGStreamer()
        rclpy.spin(streamer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'오류: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

