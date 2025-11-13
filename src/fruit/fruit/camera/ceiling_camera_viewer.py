#!/usr/bin/env python3
"""
천장 카메라 이미지를 보여주는 간단한 ROS2 노드
Simple ROS2 node to view ceiling camera images
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CeilingCameraViewer(Node):
    def __init__(self):
        super().__init__('ceiling_camera_viewer')
        
        # 파라미터 선언
        self.declare_parameter('camera_topic', '/ceiling/ceiling_camera/image_raw')
        
        # 파라미터 가져오기
        camera_topic = self.get_parameter('camera_topic').value
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # 이미지 구독자 생성
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # OpenCV 윈도우 생성
        self.window_name = 'Ceiling Camera View'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800, 600)
        
        self.get_logger().info('천장 카메라 뷰어가 시작되었습니다.')
        self.get_logger().info(f'구독 토픽: {camera_topic}')
        self.get_logger().info('종료하려면 OpenCV 창에서 "q" 키를 누르세요.')
        
        # 통계 정보
        self.frame_count = 0
        
    def image_callback(self, msg):
        """이미지 콜백 - ROS Image를 OpenCV로 변환하여 표시"""
        try:
            # ROS Image를 OpenCV 이미지로 변환
            if msg.encoding in ['rgb8', 'bgr8']:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            self.frame_count += 1
            
            # 정보 표시
            self.add_info_overlay(cv_image, msg)
            
            # 이미지 표시
            cv2.imshow(self.window_name, cv_image)
            
            # 키 입력 대기
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('종료 중...')
                cv2.destroyAllWindows()
                rclpy.shutdown()
            elif key == ord('s'):
                # 's' 키로 이미지 저장
                filename = f'ceiling_camera_{self.frame_count}.png'
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f'이미지 저장: {filename}')
                
        except Exception as e:
            self.get_logger().error(f'이미지 처리 중 오류: {str(e)}')
    
    def add_info_overlay(self, image, msg):
        """이미지에 정보 오버레이 추가"""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        color = (0, 255, 0)  # 녹색
        
        # 정보 텍스트
        info_lines = [
            f'Frame: {self.frame_count}',
            f'Size: {msg.width}x{msg.height}',
            f'Encoding: {msg.encoding}',
            f'Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}',
        ]
        
        # 텍스트 그리기
        y_offset = 30
        for text in info_lines:
            # 텍스트 크기 계산
            (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)
            
            # 반투명 배경 그리기
            overlay = image.copy()
            cv2.rectangle(overlay, (5, y_offset - text_height - 5), 
                         (15 + text_width, y_offset + 5), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)
            
            # 텍스트 그리기
            cv2.putText(image, text, (10, y_offset), 
                       font, font_scale, color, thickness)
            y_offset += 30
        
        # 중앙 십자선 그리기
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height // 2
        cross_size = 30
        cross_color = (0, 0, 255)  # 빨간색
        cv2.line(image, (center_x - cross_size, center_y), 
                (center_x + cross_size, center_y), cross_color, 2)
        cv2.line(image, (center_x, center_y - cross_size), 
                (center_x, center_y + cross_size), cross_color, 2)
        
        # 도움말 텍스트
        help_text = "Press 'q' to quit, 's' to save image"
        cv2.putText(image, help_text, (10, height - 20),
                   font, 0.5, (255, 255, 255), 1)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        viewer = CeilingCameraViewer()
        rclpy.spin(viewer)
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

