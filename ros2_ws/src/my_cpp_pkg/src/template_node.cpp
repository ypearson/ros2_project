#include <string_view>

#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/int64.hpp"
// using msg_type = std_msgs::msg::Int64;

namespace {
std::string_view sv_name{"node_name"};
std::string_view topic_name{"channel_name"};
}  // namespace

class MyCustomNode : public rclcpp::Node {
 public:
  MyCustomNode() : Node(sv_name.data()) {
    // publisher_ = this->create_publisher<msg_type>(topic_name.data(), 10);
    RCLCPP_INFO(this->get_logger(), "%s has started...", sv_name.data());
  }

 private:
  // rclcpp::Publisher<msg_type>::SharedPtr publisher_;
  // rclcpp::Subscription<msg_type>::SharedPtr subscription_;
  // rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyCustomNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}