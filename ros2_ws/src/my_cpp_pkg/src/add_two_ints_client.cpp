#include <string_view>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using Srv_msg = example_interfaces::srv::AddTwoInts;
using Request = example_interfaces::srv::AddTwoInts_Request;
using Response = example_interfaces::srv::AddTwoInts_Response;

namespace {
std::string_view sv_name{"add_two_ints_client"};
}  // namespace

class AddTwoIntsClientNode : public rclcpp::Node {
 public:
  AddTwoIntsClientNode() : Node(sv_name.data()) {
    RCLCPP_INFO(this->get_logger(), "%s has started...", sv_name.data());

    thread1 = std::thread(
      std::bind(&AddTwoIntsClientNode::callAddTwoIntsService,
      this, 1, 2));
  }
  ~AddTwoIntsClientNode() {
    if(thread1.joinable()) {
      thread1.join();
    }
  }
  void callAddTwoIntsService(int a, int b) {
    client_ = this->create_client<Srv_msg>("add_two_ints");
    while(!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Waiting for service...");
    }
    auto request = std::make_shared<Request>();
    request->a = 5;
    request->b = 5;

    auto future = client_->async_send_request(request);

    try {
      auto response = future.get(); // Blocking, spin(node) never runs
      RCLCPP_INFO(this->get_logger(), "%d + %d = %ld", a, b, response->sum);
    } catch(const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed.");
    }



  }

 private:
 std::thread thread1;
 std::vector<std::thread> threads_;
 rclcpp::Client<Srv_msg>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}