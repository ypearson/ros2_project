#include "rclcpp/rclcpp.hpp"

// Add path `/opt/ros/humble/include/**` to vs code path

class MyNode: public rclcpp::Node {
    public:
    MyNode(const char* name): Node(name) {
        RCLCPP_INFO(this->get_logger(), name);
        timer = this->create_wall_timer(std::chrono::seconds(1),
                                        std::bind(&MyNode::timerCallback, this));
    }

    private:
    int counter{};
    rclcpp::TimerBase::SharedPtr timer;
    void timerCallback(void) {
        RCLCPP_INFO(this->get_logger(), "timerCallback %d", counter++);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>("cpp_test");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}