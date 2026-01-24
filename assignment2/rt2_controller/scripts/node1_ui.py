# NODE 1 -User Interface Node for Robot Control:
#   read the keyboard input without Enter
#   get the commands from the read input
#   publish the commands to the controller node
#   call services to set threshold and get averages

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

h - help
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
        self.sub_obstacle_info = self.create_subscription(ObstacleInfo, '/obstacle_info', self.obstacle_info_callback, 10)
        
        self.lin_speed = 0.5
        self.ang_speed = 1.0

        print(HELP)
        print(f"Current speeds: linear= {self.lin_speed:.2f} m/s, angular= {self.ang_speed:.2f} rad/s")
        print("Press a key ('h' to reprint the options).")

        #to run loop
        self.should_quit = False 



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
            self.get_logger().error(f"Error in obstacle info callback: {e}")

    #callback for threshold setting
    def threshold_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(response.message)
        except Exception as e:
            self.get_logger().error(f"Error in threshold setting: {e}")
    
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
        while rclpy.ok() and not self.should_quit:
            #process incoming messages
            rclpy.spin_once(self, timeout_sec=0.1)

            key = read_key()

            if not key:
                continue

            #setting
            if key == 'q':
                print("Quit\n")
                twist = Twist()
                self.pub_cmd.publish(twist)
                self.should_quit = True 
                rclpy.shutdown()               
                break

            #help
            elif key == 'h':
                print(HELP)
                print(f"Current speeds: linear={self.lin_speed:.2f} m/s, angular={self.ang_speed:.2f} rad/s\n")

            #set threshold
            elif key == 't':
                fd = sys.stdin.fileno() #restore normal terminal for input
                old_settings = termios.tcgetattr(fd)
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                
                try:
                    val = float(input("Enter new threshold [m]: "))
                    req = SetThreshold.Request()
                    req.threshold = val
                    future = self.client_threshold.call_async(req)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                    if future.done():
                        self.threshold_callback(future)
                    else:
                        print("timeout\n")
                except ValueError:
                    print("Invalid value\n")
                except Exception as e:
                    print(f"Error in threshold setting: {e}\n")

            #get averages
            elif key == 'y':
                req = GetAverages.Request()
                future = self.client_averages.call_async(req)
                rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
                if future.done():
                    self.averages_callback(future)
                else:
                    print("timeout\n")

            #set linear speed
            elif key == 'u':
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                
                try:
                    val = float(input("Enter linear speed [m/s]: "))
                    self.lin_speed = val
                    print(f"New linear speed: {self.lin_speed:.2f} m/s\n")
                except ValueError:
                    print("Invalid value\n")

            #set angular speed
            elif key == 'i':
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                
                try:
                    val = float(input("Enter angular speed [rad/s]: "))
                    self.ang_speed = val
                    print(f"New angular speed: {self.ang_speed:.2f} rad/s\n")
                except ValueError:
                    print("Invalid value\n")

            #forward
            elif key == 'f':
                twist = Twist()
                twist.linear.x = -self.lin_speed
                twist.linear.x = self.lin_speed
                twist.angular.z = 0.0
                self.pub_cmd.publish(twist)
                print(f"Go forward: {self.lin_speed:.2f} m/s")

            #backward
            elif key == 's':
                twist = Twist()
                twist.linear.x = -self.lin_speed
                twist.angular.z = 0.0
                self.pub_cmd.publish(twist)
                print(f"Go backward: {-self.lin_speed:.2f} m/s")

            #rotate to left
            elif key == 'e':
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = self.ang_speed
                self.pub_cmd.publish(twist)
                print(f"Rotate left: {self.ang_speed:.2f} rad/s")

            #rotatwe to right
            elif key == 'r':
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = -self.ang_speed
                print(f"[DEBUG] Publishing: linear.x={twist.linear.x}, angular.z={twist.angular.z}") 
                self.pub_cmd.publish(twist)
                print(f"Rotate right: {-self.ang_speed:.2f} rad/s")

            #stop
            elif key == 'd':
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.pub_cmd.publish(twist)
                print("Stop")

            else:
                print(f"Unknown key '{key}'. Press 'h' for help.")


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