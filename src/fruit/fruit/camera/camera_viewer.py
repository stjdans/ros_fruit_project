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
import os
from ultralytics import YOLO
# 모델 파일 경로 설정 (ROS2 share 폴더 사용)
from ament_index_python.packages import get_package_share_directory


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # CV Bridge 초기화 (ROS Image <-> OpenCV 변환)
        self.bridge = CvBridge()
        
        # 카메라 정보 저장
        self.camera_info = None
        
        # YOLO 모델 초기화
        self.yolo_model = None
        self.init_yolo_model()
        
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
        
    def init_yolo_model(self):
        """YOLO 모델 초기화"""
        try:

            package_share_directory = get_package_share_directory('fruit')
            model_path = os.path.join(package_share_directory, 'config', 'yolo_model', 'fruits.pt')
            
            if os.path.exists(model_path):
                self.yolo_model = YOLO(model_path)
                self.get_logger().info(f'YOLO load success: {model_path}')
            else:
                self.get_logger().warn(f'YOLO load fail: {model_path}')
                self.yolo_model = None
                
        except Exception as e:
            self.get_logger().error(f'YOLO load fail Exception : {str(e)}')
            self.yolo_model = None
    
    def detect_fruits(self, image):
        """YOLO를 사용하여 과일 탐지"""
        if self.yolo_model is None:
            return image, []
        
        try:
            # YOLO 탐지 실행
            results = self.yolo_model(image, verbose=False)
            
            # 탐지 결과 처리
            detections = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # 바운딩 박스 좌표 추출
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        confidence = box.conf[0].cpu().numpy()
                        class_id = int(box.cls[0].cpu().numpy())
                        class_name = self.yolo_model.names[class_id]
                        
                        detections.append({
                            'bbox': [int(x1), int(y1), int(x2), int(y2)],
                            'confidence': float(confidence),
                            'class_id': class_id,
                            'class_name': class_name
                        })
            
            self.get_logger().info(f'detections : {detections}')
            return image, detections
            
        except Exception as e:
            self.get_logger().error(f'YOLO 탐지 중 오류 발생: {str(e)}')
            return image, []
    
    def draw_detections(self, image, detections):
        """탐지 결과를 이미지에 그리기"""
        for detection in detections:
            self.get_logger().info(f'detection : {detection}')
            x1, y1, x2, y2 = detection['bbox']
            confidence = detection['confidence']
            class_name = detection['class_name']
            
            # 바운딩 박스 그리기
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 라벨 텍스트
            label = f'{class_name}: {confidence:.2f}'
            
            # 텍스트 배경 사각형
            (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(image, (x1, y1 - text_height - 10), (x1 + text_width, y1), (0, 255, 0), -1)
            
            # 텍스트 그리기
            cv2.putText(image, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        return image
        
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
            
            # YOLO 탐지 실행
            cv_image, detections = self.detect_fruits(cv_image)
            
            # 탐지 결과를 이미지에 그리기
            cv_image = self.draw_detections(cv_image, detections)
            
            # 이미지 정보를 화면에 표시
            self.display_image_with_info(cv_image, msg, detections)
            
            # 'q' 키를 누르면 종료
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('종료 중...')
                cv2.destroyAllWindows()
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'이미지 변환 중 오류 발생: {str(e)}')
    
    def display_image_with_info(self, cv_image, msg, detections=None):
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
        
        # 탐지 정보 추가
        if detections:
            info_text.append(f'Detections: {len(detections)}')
            for i, detection in enumerate(detections[:3]):  # 최대 3개만 표시
                info_text.append(f'  {detection["class_name"]}: {detection["confidence"]:.2f}')
        
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

