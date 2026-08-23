#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class DemoMotionNode : public rclcpp::Node {
  public:
    DemoMotionNode()
        : Node("demo_motion_node") {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel", 10);
        timer_     = this->create_wall_timer(20ms, std::bind(&DemoMotionNode::onTimer, this));

        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Starting demo: 10s straight -> 2s right turn -> stop.");
    }

  private:
    void onTimer() {
        auto now           = this->now();
        const auto elapsed = now - start_time_;

        geometry_msgs::msg::TwistStamped twist_stamped;
        twist_stamped.twist.linear.x  = 0.0;
        twist_stamped.twist.linear.y  = 0.0;
        twist_stamped.twist.linear.z  = 0.0;
        twist_stamped.twist.angular.x = 0.0;
        twist_stamped.twist.angular.y = 0.0;
        twist_stamped.twist.angular.z = 0.0;

        if (elapsed < 10s) {
            twist_stamped.twist.linear.x  = 0.6;
            twist_stamped.twist.angular.z = 0.0;
        } else if (elapsed < 12s) {
            twist_stamped.twist.linear.x  = 0.3;
            twist_stamped.twist.angular.z = -0.35;
        } else {
            twist_stamped.twist.linear.x  = 0.0;
            twist_stamped.twist.angular.z = 0.0;
            timer_->cancel();
            RCLCPP_INFO(this->get_logger(), "Demo motion finished.");
        }

        twist_stamped.header.stamp = this->now();
        twist_stamped.header.frame_id = "base_link";

        publisher_->publish(twist_stamped);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DemoMotionNode>());
    rclcpp::shutdown();
    return 0;
}
