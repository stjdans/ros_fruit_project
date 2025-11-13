import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_control')
        self.publisher_ = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)

    def control_gripper(self, open_gripper: bool):
        msg = JointTrajectory()
        msg.joint_names = ['gripper']  # OpenMANIPULATOR-X에서는 'gripper' 또는 'gripper_joint' 사용

        point = JointTrajectoryPoint()
        if open_gripper:
            point.positions = [0.01]   # 열림 (단위: rad or m, 로봇 모델에 따라 조정 필요)
        else:
            point.positions = [0.0]    # 닫힘

        point.time_from_start.sec = 1
        msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info(f"Gripper {'opened' if open_gripper else 'closed'}")

def main(args=None):
    rclpy.init(args=args)
    node = GripperControl()

    # 예시 동작: 닫기 → 2초 후 열기
    import time
    node.control_gripper(open_gripper=False)
    time.sleep(2)
    node.control_gripper(open_gripper=True)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
