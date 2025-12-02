import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class UINode(Node):
    def __init__(self):
        super().__init__("ui_node")
        self.get_logger().info("UI node started")

        self.pub_t1 = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pub_t2 = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)


def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
