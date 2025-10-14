#!/usr/bin/env python3
"""
카메라 이미지를 OpenCV로 변환하여 보여주는 ROS2 노드
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # CV Bridge 초기화 (ROS Image <-> OpenCV 변환)
        self.bridge = CvBridge()
        
        # 카메라 정보 저장
        self.camera_info = None
        
        # 구독자 생성
        self.image_sub = self.create_subscription(
            Image,
            '/pi_camera/image_raw',
            self.image_callback,
            10)
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.camera_info_callback,
            10)
        
        # 윈도우 이름
        self.window_name = 'Camera View'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        self.get_logger().info('카메라 뷰어 노드가 시작되었습니다.')
        self.get_logger().info('이미지 토픽: /image_raw')
        self.get_logger().info('카메라 정보 토픽: /camera_info')
        self.get_logger().info('종료하려면 OpenCV 창에서 "q" 키를 누르세요.')
        
    def camera_info_callback(self, msg):
        """카메라 정보 콜백"""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(f'카메라 정보 받음:')
            self.get_logger().info(f'  해상도: {msg.width}x{msg.height}')
            self.get_logger().info(f'  왜곡 모델: {msg.distortion_model}')
            if len(msg.k) > 0:
                self.get_logger().info(f'  내부 파라미터 K: {msg.k}')
    
    def image_callback(self, msg):
        """이미지 콜백 - ROS Image를 OpenCV로 변환"""
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            if msg.encoding == 'rgb8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            elif msg.encoding == 'bgr8':
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 이미지 정보를 화면에 표시
            self.display_image_with_info(cv_image, msg)
            
            # 'q' 키를 누르면 종료
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('종료 중...')
                cv2.destroyAllWindows()
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'이미지 변환 중 오류 발생: {str(e)}')
    
    def display_image_with_info(self, cv_image, msg):
        """이미지에 정보를 추가하여 표시"""
        # 이미지 복사본 생성
        display_image = cv_image.copy()
        
        # 이미지 정보 텍스트 추가
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        font_color = (0, 255, 0)  # 녹색
        thickness = 2
        
        # 배경 사각형 (텍스트 가독성 향상)
        info_text = [
            f'Size: {msg.width}x{msg.height}',
            f'Encoding: {msg.encoding}',
            f'Frame: {msg.header.frame_id}',
            f'Seq: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
        ]
        
        y_offset = 30
        for i, text in enumerate(info_text):
            # 텍스트 크기 계산
            (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)
            
            # 배경 사각형 그리기
            cv2.rectangle(display_image, 
                         (5, y_offset - text_height - 5), 
                         (15 + text_width, y_offset + 5),
                         (0, 0, 0), -1)
            
            # 텍스트 그리기
            cv2.putText(display_image, text, (10, y_offset), 
                       font, font_scale, font_color, thickness)
            y_offset += 30
        
        # 중앙에 십자선 그리기
        height, width = display_image.shape[:2]
        center_x, center_y = width // 2, height // 2
        cross_size = 20
        cross_color = (0, 0, 255)  # 빨간색
        cv2.line(display_image, 
                (center_x - cross_size, center_y), 
                (center_x + cross_size, center_y), 
                cross_color, 2)
        cv2.line(display_image, 
                (center_x, center_y - cross_size), 
                (center_x, center_y + cross_size), 
                cross_color, 2)
        
        # 이미지 표시
        cv2.imshow(self.window_name, display_image)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_viewer = CameraViewer()
        rclpy.spin(camera_viewer)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'오류 발생: {e}')
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

