#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class DemoMotionNode : public rclcpp::Node
{
public:
  DemoMotionNode()
  : Node("demo_motion_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(20ms, std::bind(&DemoMotionNode::onTimer, this));

    start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "Starting demo: 10s straight -> 2s right turn -> stop.");
  }

private:
  void onTimer()
  {
    auto now = this->now();
    const auto elapsed = now - start_time_;

    geometry_msgs::msg::Twist twist;
    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;

    if (elapsed < 10s) {
      twist.linear.x = 0.25;
      twist.angular.z = 0.0;
    } else if (elapsed < 12s) {
      twist.linear.x = 0.10;
      twist.angular.z = -0.35;
    } else {
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
      timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "Demo motion finished.");
    }

    publisher_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DemoMotionNode>());
  rclcpp::shutdown();
  return 0;
}
