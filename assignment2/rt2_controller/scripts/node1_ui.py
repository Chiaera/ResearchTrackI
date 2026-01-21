#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rt2_interfaces.msg import Motion
from rt2_interfaces.srv import SetThreshold, GetAverages

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
        self.sub_motion = self.create_subscription(
            Motion,
            '/motion',
            self.motion_callback,
            10
        )
        
        self.get_logger().info("Insert linear velocity (linear_x) and angular velocity (angular_z):")


    # CALLBACKS
    # callback for motion info
    def motion_callback(self, msg):
        try:
            self.get_logger().info(
                f"[Motion] Average={msg.average_distance:.2f}, "
                f"min={msg.minimum_distance:.2f}, "
                f"move={msg.move}, "
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
                f"Averages: avg={response.average:.2f}, "
                f"min={response.minimum:.2f}, "
                f"msg={response.message}"
            )
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


    def run(self):
        while rclpy.ok():
            #process incoming messages
            rclpy.spin_once(self, timeout_sec=0.1)

            try:
                print("\nSET VELOCITIES:")
                lin = float(input("linear_x: "))
                ang = float(input("angular_z: "))

                twist = Twist()
                twist.linear.x = lin
                twist.angular.z = ang

                self.pub_cmd.publish(twist)
                print(f"Command sent: linear_x={lin}, angular_z={ang}")

                #request to the user
                while  (True):
                    print("\nOPTIONS:")
                    print("1 - Set new threshold")
                    print("2 - Get averages")
                    print("3 - Continue")
                    request = int(input("Choose an option: "))
                    
                    if request == 1:
                        req = SetThreshold.Request()
                        req.threshold = float(input("Insert new threshold value: "))
                        future = self.client_threshold.call_async(req)
                        future.add_done_callback(self.threshold_callback)
                        break

                    elif request == 2:
                        req = GetAverages.Request()
                        future = self.client_averages.call_async(req)
                        future.add_done_callback(self.averages_callback)
                        break
                    
                    elif request == 3:
                        break
                    
                    else:
                        print("Invalid option, try again.\n")
                    
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