#include "rclcpp/rclcpp.hpp"
#include "arm_control/msg/pos_cmd.hpp"

class PosCmdPublisher : public rclcpp::Node {
public:
  PosCmdPublisher() : Node("pos_cmd_publisher") {
    publisher_left_ = this->create_publisher<arm_control::msg::PosCmd>("ARX_VR_L", 10);
    publisher_right_ = this->create_publisher<arm_control::msg::PosCmd>("ARX_VR_R", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&PosCmdPublisher::publish_message, this));
  }

private:
  void publish_message() {
    auto message = arm_control::msg::PosCmd();
    message.x = 1.0;
    message.y = 2.0;
    // 初始化其他字段...

    publisher_left_->publish(message);
    publisher_right_->publish(message);
  }

  rclcpp::Publisher<arm_control::msg::PosCmd>::SharedPtr publisher_left_;
  rclcpp::Publisher<arm_control::msg::PosCmd>::SharedPtr publisher_right_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PosCmdPublisher>());
  rclcpp::shutdown();
  return 0;
}

