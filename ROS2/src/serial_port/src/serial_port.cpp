#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_port.hpp>
#include "vr_serial/serial_parser.hpp"
#include <iostream>
#include <dirent.h>
#include "arm_control/msg/pos_cmd.hpp"  // 包含自定义消息

// 获取所有串口设备
std::vector<std::string> get_serial_ports() {
    std::vector<std::string> ports;
    DIR *dir = opendir("/dev");
    if (dir) {
        struct dirent *entry;
        while ((entry = readdir(dir)) != nullptr) {
            if (entry->d_type == DT_CHR) {
                std::string dev_name = entry->d_name;
                if (dev_name.rfind("ttyACM", 0) == 0 || dev_name.rfind("ttyUSB", 0) == 0) {
                    ports.push_back("/dev/" + dev_name);
                }
            }
        }
        closedir(dir);
    }
    return ports;
}

class SerialPortNode : public rclcpp::Node {
public:
    SerialPortNode() : Node("serial_port_node") {
        
        // 创建发布者
        vr_right_pub_ = this->create_publisher<arm_control::msg::PosCmd>("/ARX_VR_R", 10);
        vr_left_pub_ = this->create_publisher<arm_control::msg::PosCmd>("/ARX_VR_L", 10);

        // 自动寻找串口设备
        std::vector<std::string> serial_ports = get_serial_ports();
        if (serial_ports.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No serial devices found.");
            rclcpp::shutdown();
        }

        // 假设选择第一个串口设备
        std::string port = serial_ports[0];

        // 初始化 SerialPort
        Spc = std::make_unique<drivers::serial_driver::SerialPort>(
            ctx, port, drivers::serial_driver::SerialPortConfig(
                921600,  // 波特率
                drivers::serial_driver::FlowControl::NONE,
                drivers::serial_driver::Parity::NONE,
                drivers::serial_driver::StopBits::ONE
            )
        );

        Spc->open();

        if (!Spc->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            rclcpp::shutdown();
        } else {
            RCLCPP_INFO(this->get_logger(), "%s is opened.", port.c_str());
        }

        // 定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2), std::bind(&SerialPortNode::readSerialData, this));
    }

private:
    void readSerialData() {
        if (!Spc->is_open()) return;

        try {
            std::vector<uint8_t> buffer(1024);
            size_t n = Spc->receive(buffer);
            if (n > 93) {
                if (buffer[0] == 0x55 && buffer[1] == 0xAA) { // 检验帧头
                    arm_control::msg::PosCmd msg_right;
                    arm_control::msg::PosCmd msg_left;

                    double right_x, right_y, right_z, right_roll, right_pitch, right_yaw, right_gripper;
                    double left_x, left_y, left_z, left_roll, left_pitch, left_yaw, left_gripper;
                    double chx, chy, chz, height, head_pit, head_yaw;
                    uint8_t mode1, mode2;

                    parseAndPublish(buffer.data(), right_x, right_y, right_z, right_roll, right_pitch, right_yaw, right_gripper,
                                    left_x, left_y, left_z, left_roll, left_pitch, left_yaw, left_gripper,
                                    chx, chy, chz, height, head_pit, head_yaw, mode1, mode2);

                    // 填充 ROS2 消息
                    msg_right.x = right_x;
                    msg_right.y = right_y;
                    msg_right.z = right_z;
                    msg_right.roll = right_roll;
                    msg_right.pitch = right_pitch;
                    msg_right.yaw = right_yaw;
                    msg_right.gripper = right_gripper;

                    msg_left.x = left_x;
                    msg_left.y = left_y;
                    msg_left.z = left_z;
                    msg_left.roll = left_roll;
                    msg_left.pitch = left_pitch;
                    msg_left.yaw = left_yaw;
                    msg_left.gripper = left_gripper;

                    msg_left.chx = chx;
                    msg_left.chy = chy;
                    msg_left.chz = chz;
                    msg_left.height = height;
                    msg_left.head_pit = head_pit;
                    msg_left.head_yaw = head_yaw;
                    msg_left.mode1 = mode1;
                    msg_left.mode2 = mode2;

                    vr_right_pub_->publish(msg_right);
                    vr_left_pub_->publish(msg_left);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Frame HEAD check failed: %02X %02X", buffer[0], buffer[1]);
                }
            }
        } catch (const std::exception &ex) {
            RCLCPP_WARN(this->get_logger(), "Error while receiving data: %s", ex.what());
        }
    }

    rclcpp::Publisher<arm_control::msg::PosCmd>::SharedPtr vr_right_pub_;
    rclcpp::Publisher<arm_control::msg::PosCmd>::SharedPtr vr_left_pub_;
    std::unique_ptr<drivers::serial_driver::SerialPort> Spc;  // 使用 unique_ptr 来管理 SerialPort 对象
    IoContext ctx;  // 添加 IoContext 成员
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialPortNode>());
    rclcpp::shutdown();
    return 0;
}