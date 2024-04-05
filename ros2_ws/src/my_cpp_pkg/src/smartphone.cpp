#include <string_view>

#include "example_interfaces/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

namespace {
std::string_view sv_name{"smartphone"};
}

class SmartphoneNode : public rclcpp::Node {
 public:
  SmartphoneNode() : Node(sv_name.data()) {
    RCLCPP_INFO(this->get_logger(), "%s has started...", sv_name.data());
    subscriber_ = this->create_subscription<example_interfaces::msg::String>(
        "robot_news", 10,
        std::bind(&SmartphoneNode::callbackRobotNews, this,
                  std::placeholders::_1));
  }

 private:
  void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
  }
  rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SmartphoneNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}