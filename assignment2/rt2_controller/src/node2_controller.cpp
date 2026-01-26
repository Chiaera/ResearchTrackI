// NODE 2 - Controller Node for Robot Safety and Obstacle Avoidance
//   subscribe to user commands, laser scan
//   publish safe commands to robot, obstacle avoidance
//   provide service to set obstacle threshold and get average velocities

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rt2_interfaces/msg/obstacle_info.hpp"
#include "rt2_interfaces/srv/get_averages.hpp"
#include "rt2_interfaces/srv/set_threshold.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Controller : public rclcpp::Node {
public:
  Controller() : Node("node2_controller"), threshold_(0.5) {
    // subscriber to the user command
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_input", 10, std::bind(&Controller::cmdCallback, this, _1));

    // subscribet to laser
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Controller::scanCallback, this, _1));

    // publisher of the twist
    pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // publisher of obstacle info
    pub_info_ = create_publisher<rt2_interfaces::msg::ObstacleInfo>(
        "/obstacle_info", 10);

    // setting threshold
    srv_set_threshold_ = create_service<rt2_interfaces::srv::SetThreshold>(
        "/set_threshold", std::bind(&Controller::setThreshold, this, _1, _2));

    // get averages
    srv_get_averages_ = create_service<rt2_interfaces::srv::GetAverages>(
        "/get_averages", std::bind(&Controller::getAverages, this, _1, _2));

    // timer for backward motion (100ms period)
    backup_timer_ =
        create_wall_timer(std::chrono::milliseconds(100),
                          std::bind(&Controller::backupTimerCallback, this));

    RCLCPP_INFO(get_logger(), "Controller awakes");
  }

private:
  // save the last command
  geometry_msgs::msg::Twist last_cmd_;
  std::deque<geometry_msgs::msg::Twist> last_inputs_;

  // threshold
  double threshold_;

  // backup state
  bool is_backing_up_ = false;
  rclcpp::Time backup_start_time_;
  static constexpr double BACKUP_DURATION = 1.0; // 1s
  static constexpr double BACKUP_SPEED = -0.5;   //-0.5 m/s

  // subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;

  // publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;
  rclcpp::Publisher<rt2_interfaces::msg::ObstacleInfo>::SharedPtr pub_info_;

  // services
  rclcpp::Service<rt2_interfaces::srv::SetThreshold>::SharedPtr
      srv_set_threshold_;
  rclcpp::Service<rt2_interfaces::srv::GetAverages>::SharedPtr
      srv_get_averages_;

  // timer for backup
  rclcpp::TimerBase::SharedPtr backup_timer_;

  // callback for user command
  void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_cmd_ = *msg; // store the last command (wait for safety check)

    last_inputs_.push_back(*msg);
    if (last_inputs_.size() > 5) {
      last_inputs_.pop_front();
    }
  }

  // callback for laser
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // lambda to update the minimum distance
    auto update_min = [&](double &current_min, double candidate) {
      if (std::isfinite(candidate) && candidate >= msg->range_min &&
          candidate <= msg->range_max && candidate < current_min) {
        current_min = candidate;
      }
    };

    // initialize zone minimums
    double min_global = std::numeric_limits<double>::infinity();
    double min_left = std::numeric_limits<double>::infinity();
    double min_front = std::numeric_limits<double>::infinity();
    double min_right = std::numeric_limits<double>::infinity();
    double min_back = std::numeric_limits<double>::infinity();

    // zone angles
    const double right_min_ang = -M_PI / 2.0; // -90°
    const double right_max_ang = -M_PI / 6.0; // -30°
    const double front_min_ang = -M_PI / 6.0; // -30°
    const double front_max_ang = M_PI / 6.0;  // +30°
    const double left_min_ang = M_PI / 6.0;   // +30°
    const double left_max_ang = M_PI / 2.0;   // +90°

    // back zone angles
    const double back_min_ang_1 = -M_PI;             // -180°
    const double back_max_ang_1 = -2.0 * M_PI / 3.0; // -120°
    const double back_min_ang_2 = 2.0 * M_PI / 3.0;  // +120°
    const double back_max_ang_2 = M_PI;              // +180°

    // process each laser point
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const double r = msg->ranges[i];
      const double ang =
          msg->angle_min + static_cast<double>(i) * msg->angle_increment;

      // minimum
      update_min(min_global, r);

      // minimum for angle zone
      if (ang >= right_min_ang && ang < right_max_ang) { //-90° to -30°
        update_min(min_right, r);
      } else if (ang >= front_min_ang && ang <= front_max_ang) { //-30° to +30°
        update_min(min_front, r);
      } else if (ang > left_min_ang && ang <= left_max_ang) { //+30° to +90°
        update_min(min_left, r);
      } else if ((ang >= back_min_ang_1 && ang <= back_max_ang_1) ||
                 (ang >= back_min_ang_2 && ang <= back_max_ang_2)) {
        update_min(min_back, r);
      }
    }

    // check for non-finite minimums
    if (!std::isfinite(min_global))
      min_global = 0.0; // block robot if no obstacle detected
    if (!std::isfinite(min_back))
      min_back = std::numeric_limits<double>::infinity(); // empty space if no
                                                          // obstacle detected

    // determine the direction of the closest obstacle
    std::string direction = "unknown";
    double best = std::numeric_limits<double>::infinity();
    if (std::isfinite(min_right) && min_right < best) {
      best = min_right;
      direction = "right";
    }
    if (std::isfinite(min_front) && min_front < best) {
      best = min_front;
      direction = "front";
    }
    if (std::isfinite(min_left) && min_left < best) {
      best = min_left;
      direction = "left";
    }
    if (std::isfinite(min_back) && min_back < best) {
      best = min_back;
      direction = "back";
    }
    if (direction == "unknown") {
      RCLCPP_WARN(get_logger(), "No valid obstacle distances detected.");
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Closest obstacle: %s (min=%.2f, thr=%.2f)",
                           direction.c_str(), min_global, threshold_);
    }

    // publish obstacle info
    rt2_interfaces::msg::ObstacleInfo info;
    info.min_distance = static_cast<float>(min_global);
    info.direction = direction;
    info.threshold = static_cast<float>(threshold_);
    pub_info_->publish(info);

    // set the safe zone
    double min_forward_zone = std::min({min_front, min_left, min_right});

    bool front_too_close = (min_forward_zone < threshold_);
    bool back_too_close = (min_back < threshold_);

    // automatic backward if going forward into obstacle
    if (front_too_close && last_cmd_.linear.x > 0.0 && !is_backing_up_) {
      RCLCPP_WARN(get_logger(),
                  "Obstacle in the front zone(%.2fm < %.2fm). Starting "
                  "automatic backward motion",
                  min_forward_zone, threshold_);
      is_backing_up_ = true;
      backup_start_time_ = this->now();
    }

    // no automatic backward - check commands
    if (!is_backing_up_) {
      geometry_msgs::msg::Twist safe_cmd = last_cmd_;

      // block forward if obstacle in forward zone
      if (front_too_close && safe_cmd.linear.x > 0.0) {
        safe_cmd.linear.x = 0.0;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                             "Blocked forward: obstacle in forward zone at "
                             "%.2fm (threshold %.2fm)",
                             min_forward_zone, threshold_);
      }

      // block backward if obstacle behind
      if (back_too_close && safe_cmd.linear.x < 0.0) {
        safe_cmd.linear.x = 0.0;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Blocked backward motion: obstacle at %.2fm (threshold %.2fm)",
            min_back, threshold_);
      } else if (safe_cmd.linear.x < 0.0) { // only backup motion
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Backward allowed: back_dist=%.2fm >= thr=%.2fm",
                             min_back, threshold_);
      }

      pub_cmd_->publish(safe_cmd);
    }
  }

  // callback for timer of backward motion
  void backupTimerCallback() {
    if (!is_backing_up_) {
      return;
    }

    auto elapsed = (this->now() - backup_start_time_).seconds();

    if (elapsed < BACKUP_DURATION) {
      // backward motion continue
      geometry_msgs::msg::Twist backup_cmd;
      backup_cmd.linear.x = BACKUP_SPEED;
      backup_cmd.angular.z = 0.0;
      pub_cmd_->publish(backup_cmd);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                           "Backward motion (%.1fs/%.1fs)", elapsed,
                           BACKUP_DURATION);
    } else {
      // backup complete - stop robot
      geometry_msgs::msg::Twist stop_cmd;
      stop_cmd.linear.x = 0.0;
      stop_cmd.angular.z = 0.0;
      pub_cmd_->publish(stop_cmd);

      is_backing_up_ = false;
      RCLCPP_INFO(get_logger(),
                  "Backup complete. Robot stopped in safe position.");
    }
  }

  // service: set threshold
  void setThreshold(
      const std::shared_ptr<rt2_interfaces::srv::SetThreshold::Request> req,
      std::shared_ptr<rt2_interfaces::srv::SetThreshold::Response> res) {
    threshold_ = req->threshold;
    res->success = true;
    res->message =
        "Threshold updated successfully to " + std::to_string(threshold_);
    RCLCPP_INFO(get_logger(), "New threshold: %.2f", threshold_);
  }

  // service: get average
  void
  getAverages(const std::shared_ptr<rt2_interfaces::srv::GetAverages::Request>,
              std::shared_ptr<rt2_interfaces::srv::GetAverages::Response> res) {
    if (last_inputs_.empty()) { // no inputs received yet
      res->avg_linear = 0.0;
      res->avg_angular = 0.0;
      res->message = "No velocity input received yet";
      RCLCPP_WARN(get_logger(), "GetAverages called but no inputs received");
      return;
    }

    double sum_linear = 0.0;
    double sum_angular = 0.0;

    RCLCPP_INFO(get_logger(),
                "Computing averages from %zu inputs:", last_inputs_.size());
    for (size_t i = 0; i < last_inputs_.size(); ++i) {
      const auto &cmd = last_inputs_[i];
      sum_linear += cmd.linear.x;
      sum_angular += cmd.angular.z;
      RCLCPP_INFO(get_logger(), "  [%zu] linear=%.2f, angular=%.2f", i,
                  cmd.linear.x, cmd.angular.z);
    }

    res->avg_linear = sum_linear / last_inputs_.size();
    res->avg_angular = sum_angular / last_inputs_.size();
    res->message = "Averages computed successfully over " +
                   std::to_string(last_inputs_.size()) + " input(s)";

    RCLCPP_INFO(get_logger(),
                "Robot average velocities over last %zu input(s): lin=%.2f "
                "m/s, ang=%.2f rad/s",
                last_inputs_.size(), res->avg_linear, res->avg_angular);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}