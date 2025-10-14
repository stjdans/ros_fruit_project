#!/usr/bin/env python3
"""
Gazebo 카메라 영상을 Flask 서버로 전송하는 ROS2 노드
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import requests
import base64
from threading import Thread
import time


class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer')
        
        # 파라미터 선언
        self.declare_parameter('flask_server_url', 'http://192.168.1.100:5000/upload')
        self.declare_parameter('streaming_fps', 10)  # 전송 프레임레이트
        self.declare_parameter('image_quality', 85)  # JPEG 품질 (0-100)
        self.declare_parameter('image_topic', '/pi_camera/image_raw')
        
        # 파라미터 가져오기
        self.flask_url = self.get_parameter('flask_server_url').value
        self.fps = self.get_parameter('streaming_fps').value
        self.quality = self.get_parameter('image_quality').value
        self.image_topic = self.get_parameter('image_topic').value
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 최신 이미지 저장
        self.latest_image = None
        self.image_lock = False
        
        # 구독자 생성
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)
        
        # 전송 스레드 시작
        self.streaming = True
        self.stream_thread = Thread(target=self.stream_loop)
        self.stream_thread.daemon = True
        self.stream_thread.start()
        
        self.get_logger().info('='*50)
        self.get_logger().info('카메라 스트리머 노드 시작')
        self.get_logger().info(f'Flask 서버 URL: {self.flask_url}')
        self.get_logger().info(f'전송 FPS: {self.fps}')
        self.get_logger().info(f'이미지 품질: {self.quality}%')
        self.get_logger().info(f'카메라 토픽: {self.image_topic}')
        self.get_logger().info('='*50)
        
    def image_callback(self, msg):
        """카메라 이미지 콜백"""
        if not self.image_lock:
            try:
                # ROS Image → OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.latest_image = cv_image
            except Exception as e:
                self.get_logger().error(f'이미지 변환 오류: {str(e)}')
    
    def stream_loop(self):
        """이미지를 Flask 서버로 전송하는 루프"""
        rate = 1.0 / self.fps
        
        while self.streaming and rclpy.ok():
            if self.latest_image is not None:
                self.image_lock = True
                success = self.send_image(self.latest_image)
                self.image_lock = False
                
                if success:
                    self.get_logger().info('이미지 전송 성공', throttle_duration_sec=5.0)
                else:
                    self.get_logger().warn('이미지 전송 실패', throttle_duration_sec=5.0)
            
            time.sleep(rate)
    
    def send_image(self, cv_image):
        """
        OpenCV 이미지를 Flask 서버로 전송
        
        두 가지 방식 제공:
        1. Base64 인코딩하여 JSON으로 전송
        2. Multipart form-data로 전송 (파일 업로드)
        """
        try:
            # JPEG로 인코딩
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
            _, buffer = cv2.imencode('.jpg', cv_image, encode_param)
            
            # 방법 1: Base64 인코딩 (JSON)
            # jpg_as_text = base64.b64encode(buffer).decode('utf-8')
            # data = {'image': jpg_as_text}
            # response = requests.post(self.flask_url, json=data, timeout=5)
            
            # 방법 2: Multipart form-data (파일 업로드) - 더 효율적
            files = {'image': ('frame.jpg', buffer.tobytes(), 'image/jpeg')}
            response = requests.post(self.flask_url, files=files, timeout=5)
            
            return response.status_code == 200
            
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'전송 오류: {str(e)}', throttle_duration_sec=10.0)
            return False
        except Exception as e:
            self.get_logger().error(f'예외 발생: {str(e)}')
            return False
    
    def destroy_node(self):
        """노드 종료"""
        self.streaming = False
        if self.stream_thread.is_alive():
            self.stream_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_streamer = CameraStreamer()
        rclpy.spin(camera_streamer)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

