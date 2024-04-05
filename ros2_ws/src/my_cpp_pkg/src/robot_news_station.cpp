#include <chrono>
#include <string_view>

#include "example_interfaces/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
std::string_view sv_name{"robot_news_station"};
}

class RobotNewsStationNode : public rclcpp::Node {
 public:
  RobotNewsStationNode() : Node(sv_name.data()) {
    publisher_ = this->create_publisher<example_interfaces::msg::String>(
        "robot_news", 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&RobotNewsStationNode::publishNews, this));
    RCLCPP_INFO(this->get_logger(), "%s has started...", sv_name.data());
  }

 private:
  void publishNews(void) {
    auto msg = example_interfaces::msg::String();
    msg.data = std::string("Hi, this is ") + robot_name + std::string(" from the robot News Station");
    publisher_->publish(msg);
  }
  const std::string robot_name{"R2D2"};
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotNewsStationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}