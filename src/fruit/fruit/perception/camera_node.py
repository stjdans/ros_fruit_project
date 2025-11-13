import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # ROS2 Image publisher
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # Gazebo 카메라 sensor plugin 토픽 구독
        self.sub = self.create_subscription(
            Image,
            '/open_manipulator/camera/image_raw',  # OpenManipulator 가상 카메라 토픽
            self.camera_callback,
            10
        )
        self.get_logger().info("Camera node initialized, subscribing to /open_manipulator/camera/image_raw")

    def camera_callback(self, msg):
        """
        Gazebo 카메라 토픽으로 들어온 Image 메시지를
        /camera/image_raw 로 그대로 publish
        """
        try:
            # ROS Image 메시지를 그대로 publish
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish camera frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
