#include <memory>
#include <algorithm>
#include <cmath>
#include <limits>
#include <deque>

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

        //publisher of obstacle info
        pub_info_ = create_publisher<rt2_interfaces::msg::ObstacleInfo>("/obstacle_info", 10);

        //setting threshold
        srv_set_threshold_ = create_service<rt2_interfaces::srv::SetThreshold>("/set_threshold", std::bind(&Controller::setThreshold, this, _1, _2));
        
        //get averages
        srv_get_averages_ = create_service<rt2_interfaces::srv::GetAverages>("/get_averages", std::bind(&Controller::getAverages, this, _1, _2));

        RCLCPP_INFO(get_logger(), "Controller awakes");
    }

private:
    //save the last command
    geometry_msgs::msg::Twist last_cmd_;
    std::deque<geometry_msgs::msg::Twist> last_inputs_;

    //threshold
    double threshold_;

    //subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

    //publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
    rclcpp::Publisher<rt2_interfaces::msg::ObstacleInfo>::SharedPtr pub_info_;

    //services
    rclcpp::Service<rt2_interfaces::srv::SetThreshold>::SharedPtr srv_set_threshold_;
    rclcpp::Service<rt2_interfaces::srv::GetAverages>::SharedPtr srv_get_averages_;

    //callback for user command
    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        last_cmd_ = *msg; //store the last command (wait for safety check)

        last_inputs_.push_back(*msg);
        if (last_inputs_.size() > 5) {
            last_inputs_.pop_front();
        }
    }

    //callback for  laser
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        //lambda to update the minimum distance
        auto update_min = [&](double &current_min, double candidate) {
            if (std::isfinite(candidate) && candidate >= msg->range_min && candidate <= msg->range_max && candidate < current_min)
            {
                current_min = candidate;
            }
        };  

        //initialize zone minimums
        double min_global = std::numeric_limits<double>::infinity();
        double min_left   = std::numeric_limits<double>::infinity();
        double min_front  = std::numeric_limits<double>::infinity();
        double min_right  = std::numeric_limits<double>::infinity();

        //zone angles
        const double right_min_ang = -M_PI / 2.0;   // -90°
        const double right_max_ang = -M_PI / 6.0;   // -30°
        const double front_min_ang = -M_PI / 6.0;   // -30°
        const double front_max_ang =  M_PI / 6.0;   // +30°
        const double left_min_ang  =  M_PI / 6.0;   // +30°
        const double left_max_ang  =  M_PI / 2.0;   // +90°

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            const double r = msg->ranges[i];
            const double ang = msg->angle_min + static_cast<double>(i) * msg->angle_increment;

            //minimum
            update_min(min_global, r);

            //minimum for angle zone
            if (ang >= right_min_ang && ang < right_max_ang) { //-90° to -30°
                update_min(min_right, r);
            } else if (ang >= front_min_ang && ang <= front_max_ang) { //-30° to +30°
                update_min(min_front, r);
            } else if (ang > left_min_ang && ang <= left_max_ang) { //+30° to +90°
                update_min(min_left, r);
            }
        }

        //check for non-finite minimums
        if (!std::isfinite(min_global)) {
            min_global = 0.0;
        }

        //determine the direction of the closest obstacle
        std::string direction = "unknown";
        double best = std::numeric_limits<double>::infinity();
        if (std::isfinite(min_right) && min_right < best) { best = min_right; direction = "right"; }
        if (std::isfinite(min_front) && min_front < best) { best = min_front; direction = "front"; }
        if (std::isfinite(min_left)  && min_left  < best) { best = min_left;  direction = "left";  }
        if (direction == "unknown") {
            RCLCPP_WARN(get_logger(), "No valid obstacle distances detected in any sector.");
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Closest obstacle: %s (min=%.2f, thr=%.2f)", direction.c_str(), min_global, threshold_);
        }

        //print messages for obstacles
        rt2_interfaces::msg::ObstacleInfo info;
        info.min_distance = static_cast<float>(min_global);
        info.direction = direction;
        info.threshold = static_cast<float>(threshold_);
        pub_info_->publish(info);

        //set the safe zone
        geometry_msgs::msg::Twist safe_cmd = last_cmd_;
        bool too_close = (min_global < threshold_);
        if (too_close && safe_cmd.linear.x > 0.0) { //blocking forward motion
            safe_cmd.linear.x = 0.0;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "Too close: blocking forward (min=%.2f < thr=%.2f). Rotation allowed.", min_global, threshold_);
        } else if (too_close) { 
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "TOO CLOSE: forward blocked, rotation/backward allowed (%.2f < %.2f)", min_global, threshold_);
        } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "SAFE: command allowed (%.2f >= %.2f)", min_global, threshold_);
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
        res->message = "Threshold updated successfully";
        RCLCPP_INFO(get_logger(), "New threshold: %.2f", threshold_);
    }

    //service: get average
    void getAverages(
        const std::shared_ptr<rt2_interfaces::srv::GetAverages::Request>,
        std::shared_ptr<rt2_interfaces::srv::GetAverages::Response> res)
    {
        if (last_inputs_.empty()) { //no inputs received yet
            res->avg_linear = 0.0;
            res->avg_angular = 0.0;
            res->message = "No velocity input received yet";
            return;
        }

        double sum_linear = 0.0;
        double sum_angular = 0.0;

        for (const auto &cmd : last_inputs_) {
            sum_linear += cmd.linear.x;
            sum_angular += cmd.angular.z;
        }

        res->avg_linear = sum_linear / last_inputs_.size();
        res->avg_angular = sum_angular / last_inputs_.size();
        res->message = "Averages computed successfully over " + std::to_string(last_inputs_.size()) + " input(s)";

        RCLCPP_INFO(get_logger(), "Robot average velocities over last %zu input(s): lin=%.2f, ang=%.2f",
            last_inputs_.size(), res->avg_linear, res->avg_angular);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller>());
    rclcpp::shutdown();
    return 0;
}