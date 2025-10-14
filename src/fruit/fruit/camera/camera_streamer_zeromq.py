#!/usr/bin/env python3
"""
ZeroMQ 기반 카메라 스트리머
- 브로커리스 메시징 (P2P)
- 초고성능
- 다양한 패턴 (PUB/SUB, REQ/REP, PUSH/PULL)
- 자동 재연결
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import zmq
import json
import time
from threading import Lock


class CameraStreamerZeroMQ(Node):
    def __init__(self):
        super().__init__('camera_streamer_zeromq')
        
        # 파라미터 선언
        self.declare_parameter('zmq_address', 'tcp://*:5555')
        self.declare_parameter('zmq_pattern', 'pub')  # pub, push, req
        self.declare_parameter('streaming_fps', 30)
        self.declare_parameter('image_quality', 85)
        self.declare_parameter('image_topic', '/pi_camera/image_raw')
        self.declare_parameter('high_water_mark', 10)  # 메시지 큐 크기
        self.declare_parameter('send_timeout', 1000)  # ms
        
        # 파라미터 가져오기
        self.zmq_address = self.get_parameter('zmq_address').value
        self.zmq_pattern = self.get_parameter('zmq_pattern').value
        self.fps = self.get_parameter('streaming_fps').value
        self.quality = self.get_parameter('image_quality').value
        self.image_topic = self.get_parameter('image_topic').value
        self.high_water_mark = self.get_parameter('high_water_mark').value
        self.send_timeout = self.get_parameter('send_timeout').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # ZeroMQ 초기화
        self.zmq_context = zmq.Context()
        
        # 소켓 타입 결정
        if self.zmq_pattern == 'pub':
            self.socket = self.zmq_context.socket(zmq.PUB)
            pattern_name = 'PUB/SUB'
        elif self.zmq_pattern == 'push':
            self.socket = self.zmq_context.socket(zmq.PUSH)
            pattern_name = 'PUSH/PULL'
        elif self.zmq_pattern == 'req':
            self.socket = self.zmq_context.socket(zmq.REQ)
            pattern_name = 'REQ/REP'
        else:
            self.get_logger().error(f'지원하지 않는 패턴: {self.zmq_pattern}')
            raise ValueError(f'Invalid pattern: {self.zmq_pattern}')
        
        # 소켓 옵션 설정
        self.socket.setsockopt(zmq.SNDHWM, self.high_water_mark)  # 송신 큐
        self.socket.setsockopt(zmq.SNDTIMEO, self.send_timeout)  # 타임아웃
        self.socket.setsockopt(zmq.LINGER, 0)  # 즉시 종료
        
        # 바인드 또는 연결
        try:
            if '*' in self.zmq_address or '0.0.0.0' in self.zmq_address:
                self.socket.bind(self.zmq_address)
                self.get_logger().info(f'✓ ZeroMQ 바인드: {self.zmq_address}')
            else:
                self.socket.connect(self.zmq_address)
                self.get_logger().info(f'✓ ZeroMQ 연결: {self.zmq_address}')
        except zmq.ZMQError as e:
            self.get_logger().error(f'✗ ZeroMQ 연결 실패: {str(e)}')
            raise
        
        # 최신 이미지
        self.latest_image = None
        self.image_lock = Lock()
        
        # 구독자
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)
        
        # 전송 타이머
        self.stream_timer = self.create_timer(1.0 / self.fps, self.stream_callback)
        
        # 통계
        self.frames_sent = 0
        self.frames_failed = 0
        self.total_bytes_sent = 0
        self.start_time = time.time()
        
        # 통계 타이머
        self.stats_timer = self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('='*60)
        self.get_logger().info('ZeroMQ 카메라 스트리머 시작')
        self.get_logger().info(f'주소: {self.zmq_address}')
        self.get_logger().info(f'패턴: {pattern_name}')
        self.get_logger().info(f'전송 FPS: {self.fps}')
        self.get_logger().info(f'이미지 품질: {self.quality}%')
        self.get_logger().info(f'HWM: {self.high_water_mark}')
        self.get_logger().info('='*60)
    
    def image_callback(self, msg):
        """카메라 이미지 콜백"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.image_lock:
                self.latest_image = cv_image.copy()
        except Exception as e:
            self.get_logger().error(f'이미지 변환 오류: {str(e)}')
    
    def stream_callback(self):
        """ZeroMQ로 이미지 전송"""
        with self.image_lock:
            if self.latest_image is None:
                return
            image_copy = self.latest_image.copy()
        
        try:
            # JPEG 인코딩
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
            success, buffer = cv2.imencode('.jpg', image_copy, encode_param)
            
            if not success:
                self.frames_failed += 1
                return
            
            image_bytes = buffer.tobytes()
            
            # 메타데이터 생성
            metadata = {
                'timestamp': time.time(),
                'frame_number': self.frames_sent,
                'width': image_copy.shape[1],
                'height': image_copy.shape[0],
                'encoding': 'jpeg',
                'size': len(image_bytes)
            }
            
            # ZeroMQ는 멀티파트 메시지 지원
            # 메타데이터와 이미지 데이터를 분리하여 전송 (효율적)
            metadata_json = json.dumps(metadata).encode('utf-8')
            
            try:
                # 멀티파트 메시지 전송
                self.socket.send_multipart(
                    [metadata_json, image_bytes],
                    flags=zmq.NOBLOCK
                )
                
                self.frames_sent += 1
                self.total_bytes_sent += len(image_bytes)
                
            except zmq.Again:
                # 큐가 가득 참 (드롭)
                self.frames_failed += 1
                self.get_logger().warn(
                    '전송 큐 가득참 - 프레임 드롭',
                    throttle_duration_sec=5.0
                )
            
        except zmq.ZMQError as e:
            self.get_logger().error(f'ZeroMQ 오류: {str(e)}')
            self.frames_failed += 1
        except Exception as e:
            self.get_logger().error(f'스트리밍 오류: {str(e)}')
            self.frames_failed += 1
    
    def print_stats(self):
        """통계 출력"""
        elapsed = time.time() - self.start_time
        if elapsed > 0:
            fps = self.frames_sent / elapsed
            success_rate = (
                self.frames_sent / (self.frames_sent + self.frames_failed) * 100
                if (self.frames_sent + self.frames_failed) > 0 else 0
            )
            bandwidth = (self.total_bytes_sent / elapsed) / 1024 / 1024  # MB/s
            
            # ZeroMQ 통계
            try:
                events = self.socket.getsockopt(zmq.EVENTS)
                can_send = 'Yes' if (events & zmq.POLLOUT) else 'No'
                
                self.get_logger().info(
                    f'[통계] 전송: {self.frames_sent} | '
                    f'실패: {self.frames_failed} | '
                    f'FPS: {fps:.2f} | '
                    f'성공률: {success_rate:.1f}% | '
                    f'대역폭: {bandwidth:.2f} MB/s | '
                    f'전송 가능: {can_send}'
                )
            except:
                self.get_logger().info(
                    f'[통계] 전송: {self.frames_sent} | '
                    f'실패: {self.frames_failed} | '
                    f'FPS: {fps:.2f} | '
                    f'성공률: {success_rate:.1f}%'
                )
    
    def destroy_node(self):
        """노드 종료"""
        if self.socket:
            self.socket.close()
        if self.zmq_context:
            self.zmq_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        streamer = CameraStreamerZeroMQ()
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

