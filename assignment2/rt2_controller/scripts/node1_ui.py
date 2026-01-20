#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rt2_interfaces.msg import ObstacleInfo

class UserInterface(Node):
    def __init__(self):
        super().__init__('node1_ui')

        # publisher
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_input', 10)

        # subscriber
        self.sub_obs = self.create_subscription(
            ObstacleInfo,
            '/obstacle_info',
            self.obstacle_callback,
            10
        )

        self.get_logger().info("Insert linear velocity (linear_x) and angular velocity (angular_z):")

    #callback for obstacle info
    def obstacle_callback(self, msg):
        self.get_logger().info(
            f"[Obstacle] min_distance={msg.min_distance:.2f} check on direction={msg.direction} threshold={msg.threshold:.2f}"
        )

    def run(self):
        while rclpy.ok():
            try:
                lin = float(input("linear_x: "))
                ang = float(input("angular_z: "))

                twist = Twist()
                twist.linear.x = lin
                twist.angular.z = ang

                self.pub_cmd.publish(twist)
                print("Command sent.\n")

            except ValueError:
                print("Invalid value, try again.\n")


def main(args=None):
    rclpy.init(args=args)
    node = UserInterface()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    
    finally: 
        node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()
