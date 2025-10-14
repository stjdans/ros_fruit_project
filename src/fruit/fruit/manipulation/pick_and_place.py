# src/manipulation/pick_and_place.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from manipulation.moveit_interface import MoveItInterface
from perception.yolo_detector import YOLODetector
import time

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')

        # MoveIt Interface 초기화
        self.moveit = MoveItInterface(arm_group_name="manipulator", gripper_group_name="gripper")

        # YOLO Detector 초기화 (오렌지, 바나나, 구아바만 감지)
        self.detector = YOLODetector()

        # ROS 토픽: 오더 기록용
        self.order_pub = self.create_publisher(String, 'order_topic', 10)

        self.get_logger().info("PickAndPlaceNode 초기화 완료")

    # ----------------- Pick & Place 기능 -----------------
    def pick(self, fruit_pose: PoseStamped):
        """과일 집기"""
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = "base_link"
        approach_pose.pose = fruit_pose.pose
        approach_pose.pose.position.z += 0.1  # 접근 높이

        self.moveit.go_to_pose(approach_pose)
        self.moveit.go_to_pose(fruit_pose)
        self.moveit.control_gripper(open_gripper=False)
        self.moveit.go_to_pose(approach_pose)

    def place(self, place_pose: PoseStamped):
        """저울 위에 놓기"""
        approach_pose = PoseStamped()
        approach_pose.header.frame_id = "base_link"
        approach_pose.pose = place_pose.pose
        approach_pose.pose.position.z += 0.1  # 접근 높이

        self.moveit.go_to_pose(approach_pose)
        self.moveit.go_to_pose(place_pose)
        self.moveit.control_gripper(open_gripper=True)
        self.moveit.go_to_pose(approach_pose)

    # ----------------- 자동 Pick & Place -----------------
    def process_fruits(self):
        """YOLO로 감지된 과일을 순차적으로 pick & place"""
        fruits = self.detector.detect()  # [{'class':'orange', 'pose':PoseStamped()}, ...]
        if not fruits:
            self.get_logger().info("과일이 감지되지 않았습니다.")
            return

        # 저울 위치 예시
        scale_pose = PoseStamped()
        scale_pose.header.frame_id = "base_link"
        scale_pose.pose.position.x = 0.0
        scale_pose.pose.position.y = 0.5
        scale_pose.pose.position.z = 0.2
        scale_pose.pose.orientation.w = 1.0

        for fruit in fruits:
            fruit_class = fruit['class']
            fruit_pose = fruit['pose']

            self.get_logger().info(f"{fruit_class} 집기 시작")
            self.pick(fruit_pose)
            self.place(scale_pose)
            self.get_logger().info(f"{fruit_class} 저울에 배치 완료")

            # ROS 토픽으로 오더 기록
            self.order_pub.publish(String(data=fruit_class))

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    try:
        while rclpy.ok():
            node.process_fruits()
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        node.get_logger().info("PickAndPlaceNode 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
