#include <memory>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rt2_interfaces/msg/obstacle_info.hpp"
#include "rt2_interfaces/srv/set_threshold.hpp"
#include "rt2_interfaces/srv/get_averages.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("node2_controller"), threshold_(0.5)
    {
        //subscriber to the user command
        sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_input", 10,
            std::bind(&Controller::cmdCallback, this, _1));

        //subscribet to laser
        sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Controller::scanCallback, this, _1));

        //publisher of the twist
        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        //publisher of obstacles info
        pub_info_ = create_publisher<rt2_interfaces::msg::ObstacleInfo>(
            "/obstacle_info", 10);

        //setting threshold
        srv_set_threshold_ = create_service<rt2_interfaces::srv::SetThreshold>(
            "/set_threshold",
            std::bind(&Controller::setThreshold, this, _1, _2));

        //get averages
        srv_get_averages_ = create_service<rt2_interfaces::srv::GetAverages>(
            "/get_averages",
            std::bind(&Controller::getAverages, this, _1, _2));

        RCLCPP_INFO(get_logger(), "Controller awakes");
    }

private:
    //save the last command
    geometry_msgs::msg::Twist last_cmd_;

    //threshold
    double threshold_;

    //to compute distance
    double sum_min_dist_ = 0.0;
    int count_ = 0;

    //subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

    //publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<rt2_interfaces::msg::ObstacleInfo>::SharedPtr pub_info_;

    //services
    rclcpp::Service<rt2_interfaces::srv::SetThreshold>::SharedPtr srv_set_threshold_;
    rclcpp::Service<rt2_interfaces::srv::GetAverages>::SharedPtr srv_get_averages_;

    //callback to user command
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_ = *msg;
    }

    //callback to  laser
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        //minimum distance
        double min_dist = *std::min_element(msg->ranges.begin(), msg->ranges.end());

        //update distances for the average
        sum_min_dist_ += min_dist;
        count_++;

        //check on the direction
        std::string direction = (min_dist < threshold_) ? "STOP" : "OK";

        //print  ObstacleInfo
        rt2_interfaces::msg::ObstacleInfo info;
        info.min_distance = min_dist;
        info.direction = direction;
        info.threshold = threshold_;
        pub_info_->publish(info);

        //set the safe zone
        geometry_msgs::msg::Twist safe_cmd = last_cmd_;
        if (min_dist < threshold_)
        {
            safe_cmd.linear.x = 0.0;
        }

        pub_cmd_->publish(safe_cmd);
    }

    //service: set threshold
    void setThreshold(
        const std::shared_ptr<rt2_interfaces::srv::SetThreshold::Request> req,
        std::shared_ptr<rt2_interfaces::srv::SetThreshold::Response> res)
    {
        threshold_ = req->threshold;
        res->success = true;
        RCLCPP_INFO(get_logger(), "New threshold: %.2f", threshold_);
    }

    //service: get average
    void getAverages(
        const std::shared_ptr<rt2_interfaces::srv::GetAverages::Request>,
        std::shared_ptr<rt2_interfaces::srv::GetAverages::Response> res)
    {
        if (count_ == 0)
        {
            res->average = 0.0;
        }
        else
        {
            res->average = sum_min_dist_ / count_;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}