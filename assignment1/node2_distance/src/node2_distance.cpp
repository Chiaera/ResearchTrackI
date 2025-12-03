/* Distance (node2)
    - publish on a topic (msg: std_msgs/Float32)
    - stop turtles if they are 'too close' (threshold)
    - stop turtles if they are too close to the boundaries
 */

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath> 


class  DistanceNode: public rclcpp::Node
{
public:
    DistanceNode() : Node("node2_distance")
    {
        RCLCPP_INFO(this->get_logger(), "Distance node started");
        //publisher
        pub_d_ = this->create_publisher<std_msgs::msg::Float32>(
            "/turtles_distance",
            10
        );

        //subscribers
        sub_t1_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose",
            10,
            std::bind(&DistanceNode::turtle1Callback, this, std::placeholders::_1)
        );

        sub_t2_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle2/pose",
            10,
            std::bind(&DistanceNode::turtle2Callback, this, std::placeholders::_1)
        );
    }


private: 
    //callbacks
    void turtle1Callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        pose1_x_ = msg->x;
        pose1_y_ = msg->y;
        new_pose1_ = true;

        RCLCPP_DEBUG(this->get_logger(),
        "Turtle1 pose: x=%.2f, y=%.2f", pose1_x_, pose1_y_);
    }

    void turtle2Callback(const turtlesim::msg::Pose::SharedPtr msg)
    {
        pose2_x_ = msg->x;
        pose2_y_ = msg->y;
        new_pose2_ = true;

        RCLCPP_DEBUG(this->get_logger(),
        "Turtle2 pose: x=%.2f, y=%.2f", pose2_x_, pose2_y_);
    }

    //publisher
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_d_;

    //subscribers
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_t1_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_t2_;

    //save status (pose)
    float pose1_x_{0.0f};
    float pose1_y_{0.0f};
    float pose2_x_{0.0f};
    float pose2_y_{0.0f};

    bool new_pose1_{false};
    bool new_pose2_{false};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}