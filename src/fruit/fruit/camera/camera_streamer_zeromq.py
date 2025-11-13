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
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import zmq
import json
import time
import os
from threading import Lock
from ultralytics import YOLO
# 모델 파일 경로 설정 (ROS2 share 폴더 사용)
from ament_index_python.packages import get_package_share_directory


class CameraStreamerZeroMQ(Node):
    def __init__(self):
        super().__init__('camera_streamer_zeromq')
        
        # 파라미터 선언
        self.declare_parameter('zmq_address', 'tcp://*:5555')
        self.declare_parameter('zmq_pattern', 'pub')  # pub, push, req
        self.declare_parameter('streaming_fps', 30)
        self.declare_parameter('image_quality', 85)
        # self.declare_parameter('image_topic', '/pi_camera/image_raw')
        self.declare_parameter('image_topic', '/ceiling/ceiling_camera/image_raw')
        self.declare_parameter('high_water_mark', 10)  # 메시지 큐 크기
        self.declare_parameter('send_timeout', 1000)  # ms
        self.declare_parameter('use_yolo', True)  # YOLO 사용 여부
        
        # 파라미터 가져오기
        self.zmq_address = self.get_parameter('zmq_address').value
        self.zmq_pattern = self.get_parameter('zmq_pattern').value
        self.fps = self.get_parameter('streaming_fps').value
        self.quality = self.get_parameter('image_quality').value
        self.image_topic = self.get_parameter('image_topic').value
        self.high_water_mark = self.get_parameter('high_water_mark').value
        self.send_timeout = self.get_parameter('send_timeout').value
        self.use_yolo = self.get_parameter('use_yolo').value
        
        # 파라미터 변경 콜백 등록
        self.add_on_set_parameters_callback(self.parameter_callback)
        
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
        self.get_logger().info(f'YOLO 사용: {self.use_yolo}')
        self.get_logger().info('='*60)
        
        # YOLO 모델 초기화
        self.yolo_model = None
        self.init_yolo_model()

    def image_callback(self, msg):
        """카메라 이미지 콜백"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if self.use_yolo :
                # YOLO 탐지 실행
                cv_image, detections = self.detect_fruits(cv_image)
                
                # 탐지 결과를 이미지에 그리기
                cv_image = self.draw_detections(cv_image, detections)
                
                # # 이미지 정보를 화면에 표시
                cv_image = self.display_image_with_info(cv_image, msg, detections)
                
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
            
            # 0.4 미만 탐지 결과는 패스
            if confidence < 0.4:
                continue
            
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
        # info_text = [
        #     f'Size: {msg.width}x{msg.height}',
        #     f'Encoding: {msg.encoding}',
        #     f'Frame: {msg.header.frame_id}',
        #     f'Seq: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
        # ]
        
        # 탐지 정보 추가
        # if detections:
        #     info_text.append(f'Detections: {len(detections)}')
        #     for i, detection in enumerate(detections[:3]):  # 최대 3개만 표시
        #         info_text.append(f'  {detection["class_name"]}: {detection["confidence"]:.2f}')
        
        y_offset = 30
        # 텍스트 그리기
        cv2.putText(display_image, "YOLO detecting", (10, y_offset), 
                       font, font_scale, font_color, thickness)
        
        # for i, text in enumerate(info_text):
        #     # 텍스트 크기 계산
        #     (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)
            
        #     # 배경 사각형 그리기
        #     cv2.rectangle(display_image, 
        #                  (5, y_offset - text_height - 5), 
        #                  (15 + text_width, y_offset + 5),
        #                  (0, 0, 0), -1)
            
        #     # 텍스트 그리기
        #     cv2.putText(display_image, text, (10, y_offset), 
        #                font, font_scale, font_color, thickness)
        #     y_offset += 30
        
        # # 중앙에 십자선 그리기
        # height, width = display_image.shape[:2]
        # center_x, center_y = width // 2, height // 2
        # cross_size = 20
        # cross_color = (0, 0, 255)  # 빨간색
        # cv2.line(display_image, 
        #         (center_x - cross_size, center_y), 
        #         (center_x + cross_size, center_y), 
        #         cross_color, 2)
        # cv2.line(display_image, 
        #         (center_x, center_y - cross_size), 
        #         (center_x, center_y + cross_size), 
        #         cross_color, 2)
        
        # # 이미지 표시
        # cv2.imshow(self.window_name, display_image)
        return display_image
    
    def parameter_callback(self, params):
        """파라미터 변경 콜백"""
        result = SetParametersResult(successful=True)
        
        for param in params:
            if param.name == 'use_yolo':
                old_value = self.use_yolo
                self.use_yolo = param.value
                self.get_logger().info(
                    f'✓ YOLO 사용 변경: {old_value} → {self.use_yolo}'
                )
                
            elif param.name == 'image_quality':
                old_value = self.quality
                self.quality = param.value
                self.get_logger().info(
                    f'✓ 이미지 품질 변경: {old_value}% → {self.quality}%'
                )
                
            elif param.name == 'streaming_fps':
                old_value = self.fps
                self.fps = param.value
                # 타이머 재생성 필요 (간단히 값만 변경)
                self.get_logger().info(
                    f'✓ FPS 변경: {old_value} → {self.fps} (재시작 필요)'
                )
        
        return result
    
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

