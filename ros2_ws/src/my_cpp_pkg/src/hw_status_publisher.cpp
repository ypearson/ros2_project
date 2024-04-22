#include <string_view>

#include "my_robot_interfaces/msg/hardware_status.hpp"
#include "rclcpp/rclcpp.hpp"

using MsgType = my_robot_interfaces::msg::HardwareStatus;

namespace {
std::string_view sv_name{"hw_status_publisher"};
std::string_view topic_name{"hw_status"};
}  // namespace

class HwStatusPublisher : public rclcpp::Node {
 public:
  HwStatusPublisher() : Node(sv_name.data()) {
    publisher_ = this->create_publisher<MsgType>(topic_name.data(), 10);
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&HwStatusPublisher::callback, this));
    RCLCPP_INFO(this->get_logger(), "%s has started...", sv_name.data());
  }

 private:
  void callback(void) {
    MsgType msg;
    msg.are_motors_ready = true;
    msg.debug_message = "debug msg...";
    msg.temperature = 99;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publish!");
  }
  rclcpp::Publisher<MsgType>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HwStatusPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}