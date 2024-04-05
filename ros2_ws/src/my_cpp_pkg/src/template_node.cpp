#include <string_view>

#include "rclcpp/rclcpp.hpp"

namespace {
std::string_view sv_name{"node_name"};
}

class MyCustomNode : public rclcpp::Node {
 public:
  MyCustomNode() : Node(sv_name.data()) {
    RCLCPP_INFO(this->get_logger(), "%s has started...", sv_name.data());
  }

 private:
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyCustomNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}