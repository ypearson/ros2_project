#include <string_view>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
using msg_type = std_msgs::msg::Int64;

namespace {
std::string_view node_name{"number_counter"};
std::string_view topic_name{"number_count"};
}  // namespace

class NumberCounterNode : public rclcpp::Node {
 public:
  NumberCounterNode() : Node(node_name.data()) {
    publisher_ = this->create_publisher<msg_type>(topic_name.data(), 10);
    subscription_ = this->create_subscription<msg_type>(
        "number", 10,
        std::bind(&NumberCounterNode::getMsg, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "%s has started...", node_name.data());
  }

 private:
  void getMsg(msg_type msg) {
    count = msg.data;
    publisher_->publish(msg);
  }
  void publish(void) {
    msg_type msg;
    msg.data = count;
    publisher_->publish(msg);
  }
  uint64_t count;
  rclcpp::Publisher<msg_type>::SharedPtr publisher_;
  rclcpp::Subscription<msg_type>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberCounterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}