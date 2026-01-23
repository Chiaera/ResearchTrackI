#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rt2_interfaces.msg import ObstacleInfo
from rt2_interfaces.srv import SetThreshold, GetAverages

import sys
import termios
import tty


HELP = r"""
f - Forward
s - Backward
e - Rotate left
r - Rotate right
d - Stop
t - Set new threshold
y - Get averages of last 5 velocities
u - Set linear speed
i - Set angular speed
q - Quit
"""

def read_key():
    """Read 1 key without Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

class UserInterface(Node):
    def __init__(self):
        super().__init__('node1_ui')

        # services
        self.client_threshold = self.create_client(SetThreshold, '/set_threshold')
        self.client_averages = self.create_client(GetAverages, '/get_averages')
        #wait fot service to be available
        while not self.client_threshold.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for /set_threshold...")
        while not self.client_averages.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("waiting for /get_averages...")

        # publisher
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_input', 10)

        # subscriber
        self.sub_obstacle_info = self.create_subscription(
            ObstacleInfo,
            '/obstacle_info',
            self.obstacle_info_callback,
            10
        )
        
        self.lin_speed = 0.5
        self.ang_speed = 1.0

        print(HELP)
        print(f"Current speeds: linear= {self.lin_speed:.2f}, angular= {self.ang_speed:.2f}")
        print("Press a key ('h' to reprint the options).")



    # CALLBACKS
    # callback for obstacle info
    def obstacle_info_callback(self, msg):
        try:
            self.get_logger().info(
                f"[ObstacleInfo] min distance={msg.min_distance:.2f}, "
                f"direction={msg.direction}, "
                f"threshold={msg.threshold:.2f}"
            )
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    #callback for threshold setting
    def threshold_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
    
    #callback for averages
    def averages_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f"{response.message}: "
                f"linear= {response.avg_linear:.2f}, "
                f"angular= {response.avg_angular:.2f}, "
            )
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


    def run(self):
        while rclpy.ok():
            #process incoming messages
            rclpy.spin_once(self, timeout_sec=0.1)

            key = read_key()

            #setting
            if key == 'q': # quit
                print("\nQuitting.")
                twist = Twist()
                self.pub_cmd.publish(twist)
                break
            elif key == 'h': # help
                print(HELP)
                continue
            elif key == 't': # set threshold
                try:
                    req = SetThreshold.Request()
                    req.threshold = float(input("\nInsert new threshold value: "))
                    future = self.client_threshold.call_async(req)
                    rclpy.spin_until_future_complete(self, future)
                    self.threshold_callback(future)
                except ValueError:
                    print("Invalid threshold.")
                continue
            elif key == 'y': # get averages
                req = GetAverages.Request()
                future = self.client_averages.call_async(req)
                rclpy.spin_until_future_complete(self, future)
                self.averages_callback(future)
                continue
            elif key == 'u': # set linear speed
                try:
                    self.lin_speed = float(input("\nSet linear speed: "))
                    print(f"new linear speed= {self.lin_speed:.2f}")
                except ValueError:
                    print("Invalid value.")
                continue
            elif key == 'i': # set angular speed
                try:
                    self.ang_speed = float(input("\nSet angular speed: "))
                    print(f"new angular speed= {self.ang_speed:.2f}")
                except ValueError:
                    print("Invalid value.")
                continue

            #motion
            elif key == 'f': # forward
                twist = Twist()
                twist.linear.x = self.lin_speed
                twist.angular.z = 0.0
                self.pub_cmd.publish(twist)
                continue
            elif key == 's':  # backward
                twist = Twist()
                twist.linear.x = -self.lin_speed
                twist.angular.z = 0.0
                self.pub_cmd.publish(twist)
                continue
            elif key == 'e':  # rotate left
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = self.ang_speed
                self.pub_cmd.publish(twist)
                continue
            elif key == 'r':  # rotate right
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = -self.ang_speed
                self.pub_cmd.publish(twist)
                continue
            elif key == 'd':  # stop
                twist = Twist()
                self.pub_cmd.publish(twist)
                continue
            else:
                print("Invalid key. Press 'h' for help.")
                continue


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