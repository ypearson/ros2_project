#include <string_view>

#include "example_interfaces/srv/set_bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"

using ServiceRequest = example_interfaces::srv::SetBool_Request::SharedPtr;
using ServiceResponse = example_interfaces::srv::SetBool_Response::SharedPtr;
using MsgType = std_msgs::msg::Int64;
using SrvType = example_interfaces::srv::SetBool;

using std::placeholders::_1;
using std::placeholders::_2;

/*
  ros2 interface show example_interfaces/srv/SetBool
  # This is an example of a service to set a boolean value.
  # This can be used for testing but a semantically meaningful
  # one should be created to be built upon.

  bool data # e.g. for hardware enabling / disabling
  ---
  bool success   # indicate successful run of triggered service
  string message # informational, e.g. for error messages
*/

// ros2 service call /reset_counter example_interfaces/srv/SetBool "{data: true}"

namespace {
std::string_view node_name{"number_counter"};
std::string_view topic_name{"number_count"};
}  // namespace

class NumberCounterNode : public rclcpp::Node {
 public:
  NumberCounterNode() : Node(node_name.data()) {
    publisher_ = this->create_publisher<MsgType>(topic_name.data(), 10);
    subscription_ = this->create_subscription<MsgType>(
        "number", 10, std::bind(&NumberCounterNode::getMsg, this, _1));
    server_ = this->create_service<SrvType>(
        "reset_counter",
        std::bind(&NumberCounterNode::callback_service, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "%s has started...", node_name.data());
  }

 private:
  void callback_service(const ServiceRequest request,
                        const ServiceResponse response) {
    if (request->data) {
      count = 0;
      response->success = true;
      response->message = "ok";
      RCLCPP_INFO(this->get_logger(), "Reset counter");
    } else {
      response->success = false;
      response->message = "nope";
      RCLCPP_INFO(this->get_logger(), "Don't reset counter");
    }
  }
  void getMsg(MsgType msg) {
    count += msg.data;
    publisher_->publish(msg);
  }
  void publish(void) {
    MsgType msg;
    msg.data = count;
    publisher_->publish(msg);
  }
  uint64_t count{};
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
  rclcpp::Publisher<MsgType>::SharedPtr publisher_;
  rclcpp::Subscription<MsgType>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberCounterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}