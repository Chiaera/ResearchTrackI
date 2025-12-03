/* Distance (node2)
    - publish on a topic (msg: std_msgs/Float32)
    - stop turtles if they are 'too close' (threshold)
    - stop turtles if they are too close to the boundaries
 */

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath> 


class  DistanceNode: public rclcpp::Node
{
public:
    DistanceNode() : Node("node2_distance")
    {
        RCLCPP_INFO(this->get_logger(), "Distance node started");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}