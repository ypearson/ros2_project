#include <string_view>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using SrvMsg = example_interfaces::srv::AddTwoInts;
using request = example_interfaces::srv::AddTwoInts_Request::SharedPtr;
using response = example_interfaces::srv::AddTwoInts_Response::SharedPtr;

using std::placeholders::_1;
using std::placeholders::_2;

namespace {
std::string_view sv_name{"add_two_ints_server"};
}  // namespace

class AddTwoIntsNode : public rclcpp::Node {
 public:
  AddTwoIntsNode() : Node(sv_name.data()) {
    server_ = this->create_service<SrvMsg>(
        "add_two_ints",
        std::bind(&AddTwoIntsNode::callbackAddTwoInts, this, _1, _2));
    RCLCPP_INFO(this->get_logger(), "%s has started...", sv_name.data());
  }

 private:
  rclcpp::Service<SrvMsg>::SharedPtr server_;
  void callbackAddTwoInts(const request req, const response res) {
    res->sum = req->a + req->b;
    RCLCPP_INFO(this->get_logger(), "%ld + %ld = %ld", req->a, req->b,
                res->sum);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}