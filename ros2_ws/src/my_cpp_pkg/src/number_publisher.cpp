#include <chrono>
#include <string_view>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using msg_type = std_msgs::msg::Int64;
namespace {
std::string_view sv_name{"number_publisher"};
std::string_view topic_name{"number"};
}  // namespace

class NumberPublisherNode : public rclcpp::Node {
 public:
  NumberPublisherNode() : Node(sv_name.data()) {
    publisher_ = this->create_publisher<msg_type>(topic_name.data(), 10);
    timer_ =
        this->create_wall_timer(std::chrono::seconds(1),
                                std::bind(&NumberPublisherNode::publish, this));
    RCLCPP_INFO(this->get_logger(), "%s has started...", sv_name.data());
  }

 private:
  rclcpp::Publisher<msg_type>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  void publish(void) {
    static int64_t counter{};
    msg_type msg;
    msg.data = counter++;
    publisher_->publish(msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}