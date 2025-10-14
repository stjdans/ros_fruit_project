import torch
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import yaml
from ultralytics import YOLO

class YOLODetector(Node):
    def __init__(self, model_path='config/yolo_model/mixed_fruits.pt',
                 classes_path='config/yolo_model/classes.yaml'):
        super().__init__('yolo_detector')

        # ROS2 카메라 이미지 구독
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # ROS2 마커 퍼블리셔 (RViz 시각화용)
        self.marker_pub = self.create_publisher(MarkerArray, 'yolo_markers', 10)

        # YOLO 모델 로드 (.pt 파일)
        self.model = YOLO(model_path)  # ultralytics YOLO 사용
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)

        # 클래스 읽기
        with open(classes_path, 'r') as f:
            self.classes = yaml.safe_load(f)['classes']

        self.get_logger().info(f'YOLO Detector initialized with classes: {self.classes}')

    def image_callback(self, msg):
        # ROS Image -> OpenCV
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        orig_h, orig_w = img.shape[:2]

        # YOLO 추론 (ultralytics API 사용)
        results = self.model(img, device=self.device, verbose=False)
        
        # 결과 추출
        if len(results) == 0 or len(results[0].boxes) == 0:
            return
        
        result = results[0]
        boxes = result.boxes.xyxy.cpu().numpy()  # [x1, y1, x2, y2]
        scores = result.boxes.conf.cpu().numpy()  # confidence scores
        class_ids = result.boxes.cls.cpu().numpy()  # class IDs

        # RViz MarkerArray 생성
        marker_array = MarkerArray()
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = box[:4]
            marker = Marker()
            marker.header.frame_id = 'camera_link'  # 카메라 프레임
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.id = i
            marker.scale.x = (x2 - x1) / orig_w  # 상대 크기
            marker.scale.y = (y2 - y1) / orig_h
            marker.scale.z = 0.05  # 임의 높이
            marker.color.r = 1.0 if class_ids[i] == 0 else 0.0
            marker.color.g = 1.0 if class_ids[i] == 1 else 0.0
            marker.color.b = 1.0 if class_ids[i] == 2 else 0.0
            marker.color.a = 0.8
            marker.pose.position.x = (x1 + x2) / 2 / orig_w
            marker.pose.position.y = (y1 + y2) / 2 / orig_h
            marker.pose.position.z = 0.025
            marker_array.markers.append(marker)

        # Publish MarkerArray
        self.marker_pub.publish(marker_array)

        # 디버그: 콘솔 출력
        for i, cid in enumerate(class_ids):
            cls_name = self.classes[int(cid)]
            print(f"Detected {cls_name} at {boxes[i]}")

    def detect(self):
        """과일 감지 결과 반환 (pick_and_place.py에서 호출)"""
        # 실제 구현에서는 최근 감지 결과를 반환해야 함
        # 현재는 임시로 빈 리스트 반환
        return []

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
