'''UI (node1)
    - input: selection turtle, insert velocity
    - execution for 1 second
    - new input -> loop '''

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class UINode(Node):
    def __init__(self):
        super().__init__("ui_node")
        self.get_logger().info("UI node started")

        #define publisher
        self.pub_t1 = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pub_t2 = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)

        #define duration of command
        self.timer = self.create_timer(0.5, self.command_loop)


    #duration of command - 1s
    def command_loop(self):
        self.timer.cancel()

        while rclpy.ok():
            turtle_id = get_turtle()
            turtle_v, turtle_w = get_twist()

            self.get_logger().info(
                f"Turtle {turtle_id} selected with "
                f"linear velocity {turtle_v}, "
                f"and angular velocity {turtle_w}"
            )

            #select publisher
            if turtle_id == 1:
                pub = self.pub_t1
            else:
                pub = self.pub_t2

            #msg
            msg = Twist()
            msg.linear.x = turtle_v
            msg.angular.z = turtle_w
            Dt = 1.0
            start = time.time()
            
            while time.time()-start < Dt and rclpy.ok():
                pub.publish(msg)
                time.sleep(0.1)
            
            msg_stop = Twist()
            pub.publish(msg_stop)
            
            self.get_logger().info("Command finished, turtle stopped! Insert a new input\n")


#select the turtle
def get_turtle():
    while True:
        try:
            turtle_input = input("Press\n   '1' to select Turtle1\n   '2' to select Turtle2\n")
            turtle_id = int(turtle_input.strip())

            if turtle_id == 1:
                print("You selected Turtle1!")
                return 1
            elif turtle_id == 2:
                print("You selected Turtle2!")
                return 2
            else:
                print("Invalid output: select 1 or 2")
        except ValueError:
            print("Invalid turtle input")


#insert the velocity
def get_twist():
    while True:
        v_max = 20.0
        try:
            linear_velocity = input("Insert the linear velocity (x-axis), :\n") 
            turtle_v = float(linear_velocity.strip())
            #check on velocity
            if (turtle_v > 20):
                print(f"Turtle too fast! Default velocity ({v_max})")
                turtle_v = float(v_max)

            angular_velocity = input("Insert the angular velocity (z-axis):\n") 
            turtle_w = float(angular_velocity.strip())

            return turtle_v, turtle_w
        except ValueError:
            print("Invalid velocity input")


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
