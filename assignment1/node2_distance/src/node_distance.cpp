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
#include "std_msgs/msg/int32.hpp"


class DistanceNode : public rclcpp::Node
{
public:
    DistanceNode() : Node("node_distance")
    {
        RCLCPP_INFO(this->get_logger(), "Distance node started");

        // publishers
        pub_d_ = this->create_publisher<std_msgs::msg::Float32>(
            "/turtles_distance",
            10
        );
        //for collision part
        pub_t1_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel",
            10
        );
        pub_t2_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle2/cmd_vel",
            10
        );

        // subscribers
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
        //for the orange zone
        sub_t1_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel",
            10,
            std::bind(&DistanceNode::turtle1CmdCallback, this, std::placeholders::_1)
        );
        sub_t2_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle2/cmd_vel",
            10,
            std::bind(&DistanceNode::turtle2CmdCallback, this, std::placeholders::_1)
        );
        //active turtle
        sub_active_ = this->create_subscription<std_msgs::msg::Int32>(
            "/active_turtle",
            10,
            std::bind(&DistanceNode::activeTurtleCallback, this, std::placeholders::_1)
        );
    }

private:
    // levels
    enum class PreventCollision { GREEN, ORANGE, RED };

    const float collision_red_    = 1.0f;
    const float collision_orange_ = 2.0f;

    PreventCollision last_border1_{PreventCollision::GREEN};
    PreventCollision last_border2_{PreventCollision::GREEN};

    int active_turtle_{0};  
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_active_;


    //collision between turtles
    PreventCollision zone_color(float d)
    {
        if (d < collision_red_) {
            return PreventCollision::RED;
        } else if (d < collision_orange_) {
            return PreventCollision::ORANGE;
        } else {
            return PreventCollision::GREEN;
        }
    }

    //collision with border
    PreventCollision border_color(float x, float y)
    {
        // red
        const float red_min = 1.0f;
        const float red_max = 10.0f;

        if (x <= red_min || x >= red_max || y <= red_min || y >= red_max) {
            return PreventCollision::RED;
        }

        // orange
        const float orange_lim = 0.5f;

        bool near_left   = (x <= red_min + orange_lim);
        bool near_right  = (x >= red_max - orange_lim);
        bool near_bottom = (y <= red_min + orange_lim);
        bool near_top    = (y >= red_max - orange_lim);

        if (near_left || near_right || near_bottom || near_top) {
            return PreventCollision::ORANGE;
        }

        // green
        return PreventCollision::GREEN;
    }

    //stop turtles
    void stop_t1()
    {
        geometry_msgs::msg::Twist stop_msg;
        pub_t1_cmd_->publish(stop_msg);
        RCLCPP_WARN(this->get_logger(), "Turtle1: STOP");
    }
    void stop_t2()
    {
        geometry_msgs::msg::Twist stop_msg;
        pub_t2_cmd_->publish(stop_msg);
        RCLCPP_WARN(this->get_logger(), "Turtle2: STOP");
    }

    void avoid_t1(){
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = -1.0;
        if (pose2_x_ > pose1_x_) {
            cmd.angular.z = -0.5;
        } else {
            cmd.angular.z = 0.5;
        }

        pub_t1_cmd_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Turtle1: avoidance manoeuvre");
    }
    void avoid_t2(){
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = -1.0;
        if (pose2_x_ > pose1_x_) {
            cmd.angular.z = 0.5;
        } else {
            cmd.angular.z = -0.5;
        }

        pub_t2_cmd_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Turtle2: avoidance manoeuvre");
    }

    void retreat_t1() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = -1.0;     
        cmd.angular.z = 0.0;
        pub_t1_cmd_->publish(cmd);
        RCLCPP_WARN(this->get_logger(), "Turtle1: retreat");
    }
    void retreat_t2() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = -1.0;
        cmd.angular.z = 0.0;
        pub_t2_cmd_->publish(cmd);
        RCLCPP_WARN(this->get_logger(), "Turtle2: retreat");
    }

    void retreat_and_turn_t1() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = -1.0;      
        cmd.angular.z = M_PI/4;   
        pub_t1_cmd_->publish(cmd);
        RCLCPP_WARN(this->get_logger(), "Turtle1: retreat + 90deg turn");
    }

    void retreat_and_turn_t2() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = -1.0;      
        cmd.angular.z = M_PI/4;    
        pub_t2_cmd_->publish(cmd);
        RCLCPP_WARN(this->get_logger(), "Turtle2: retreat + 90deg turn");
    }



    //distance
    void calculate_distance()
    {
        if (!new_pose1_ || !new_pose2_) {
            return;
        }

        // computation
        float dx = pose1_x_ - pose2_x_;
        float dy = pose1_y_ - pose2_y_;
        float d  = std::sqrt(dx * dx + dy * dy);

        // publication
        std_msgs::msg::Float32 msg;
        msg.data = d;
        pub_d_->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Distance between turtles: %.3f", d);

        //turtles distance
        PreventCollision mode = zone_color(d);

        switch (mode) {
        case PreventCollision::GREEN:
            RCLCPP_DEBUG(this->get_logger(), "The turtles are far away");
            break;

        case PreventCollision::ORANGE:
            RCLCPP_DEBUG(this->get_logger(), "The turtles are close: avoidance");
            if (active_turtle_ == 1) {
                avoid_t2();
            } else if (active_turtle_ == 2) {
                avoid_t1();
            } else { //fallback
                avoid_t2();
            }
            break;

        case PreventCollision::RED:
            RCLCPP_DEBUG(this->get_logger(), "The turtles are too close: stop");
            if (active_turtle_ == 1) {
                stop_t1();
            } else if (active_turtle_ == 2) {
                stop_t2();
            } else {
                stop_t1();
                stop_t2();
            }
        }

        //collision with borders
        PreventCollision border1 = border_color(pose1_x_, pose1_y_);
        PreventCollision border2 = border_color(pose2_x_, pose2_y_);

        if(active_turtle_==1 &&(border1 == PreventCollision::RED || border1 == PreventCollision::ORANGE)) {
            retreat_and_turn_t1();
        }
        if(active_turtle_==2 &&-(border2 == PreventCollision::RED || border2 == PreventCollision::ORANGE)) {
            retreat_and_turn_t2();
        }

        //update last pose
        last_border1_ = border1;
        last_border2_ = border2;
    }

    //callbacks
    void turtle1Callback(const turtlesim::msg::Pose::SharedPtr msg){
        pose1_x_   = msg->x;
        pose1_y_   = msg->y;
        new_pose1_ = true;

        RCLCPP_DEBUG(this->get_logger(), "Turtle1 pose: x=%.2f, y=%.2f", pose1_x_, pose1_y_);

        calculate_distance();
    }

    void turtle2Callback(const turtlesim::msg::Pose::SharedPtr msg) {
        pose2_x_   = msg->x;
        pose2_y_   = msg->y;
        new_pose2_ = true;

        RCLCPP_DEBUG(this->get_logger(), "Turtle2 pose: x=%.2f, y=%.2f", pose2_x_, pose2_y_);

        calculate_distance();
    }

    void activeTurtleCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        active_turtle_ = msg->data;
        RCLCPP_INFO(this->get_logger(), "Active turtle set to: %d", active_turtle_);
    }

    void turtle1CmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
        last_cmd_t1_ = *msg;
        new_cmd_t1_ = true;
    }

    void turtle2CmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
        last_cmd_t2_ = *msg;
        new_cmd_t2_ = true;
    }


    //publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr      pub_d_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   pub_t1_cmd_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   pub_t2_cmd_;

    //subscribers
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_t1_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_t2_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_t1_cmd_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_t2_cmd_;

    //save state (poses)
    float pose1_x_{0.0f};
    float pose1_y_{0.0f};
    float pose2_x_{0.0f};
    float pose2_y_{0.0f};

    bool new_pose1_{false};
    bool new_pose2_{false};

    geometry_msgs::msg::Twist last_cmd_t1_;
    geometry_msgs::msg::Twist last_cmd_t2_;
    bool new_cmd_t1_{false};
    bool new_cmd_t2_{false};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceNode>());
    rclcpp::shutdown();
    return 0;
}