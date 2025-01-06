// serial_port.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <serial_port/serial_port.hpp>
#include "vr_serial/serial_parser.hpp"  // 包含解析头文件
#include <iostream>
#include <pos_cmd_msg/PosCmd.h>
#include <dirent.h>  // 用于读取目录

// 获取所有串口设备
std::vector<std::string> get_serial_ports() {
    std::vector<std::string> ports;
    DIR *dir = opendir("/dev");
    if (dir) {
        struct dirent *entry;
        while ((entry = readdir(dir)) != nullptr) {
            if (entry->d_type == DT_CHR) {  // 检查是否为字符设备
                std::string dev_name = entry->d_name;
                // 只匹配ttyACM和ttyUSB设备
                if (dev_name.rfind("ttyACM", 0) == 0 || dev_name.rfind("ttyUSB", 0) == 0) {
                    ports.push_back("/dev/" + dev_name);
                }
            }
        }
        closedir(dir);
    }
    return ports;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_port");
    // 创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    // 发布话题
    ros::Publisher VRRightpub = n.advertise<pos_cmd_msg::PosCmd>("/ARX_VR_R",10);
    ros::Publisher VRleftpub = n.advertise<pos_cmd_msg::PosCmd>("/ARX_VR_L",10);

    // 创建一个serial类
    serial::Serial sp;
    // 创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    // 自动寻找串口设备
    std::vector<std::string> serial_ports = get_serial_ports();
    if (serial_ports.empty()) {
        ROS_ERROR("No serial devices found.");
        return -1;
    }
    // 假设选择第一个串口设备
    std::string port = serial_ports[0];
    sp.setPort(port);
    // 设置串口通信的波特率
    sp.setBaudrate(921600);
    // 串口设置timeout
    sp.setTimeout(to);

    pos_cmd_msg::PosCmd msgvrright;
    pos_cmd_msg::PosCmd msgvrleft;

    try
    {
        // 打开串口
        sp.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open virtual port.");
        return -1;
    }

    // 判断串口是否打开成功
    if (sp.isOpen())
    {
        // ROS_INFO_STREAM("/dev/ttyACM1 is opened.");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(500);
    uint8_t buffer[1024];

    while (ros::ok())
    {
        if (sp.isOpen())
        {
        }
        else
        {
        }
        try
        {
            // 获取缓冲区内的字节数
            size_t n = sp.available();
            if (n > 93) // 如果缓冲区有90个数据
            {
                uint8_t buffer[1024];
                // 读出数据
                n = sp.read(buffer, n);

                if (buffer[0] == 0x55 && buffer[1] == 0xAA)  // 检验帧头
                {                                             
                        double right_x, right_y, right_z, right_roll, right_pitch, right_yaw, right_gripper;
                        double left_x, left_y, left_z, left_roll, left_pitch, left_yaw, left_gripper;
                        double chx, chy, chz, height, head_pit, head_yaw;
                        uint8_t mode1, mode2;

                        parseAndPublish(buffer, right_x, right_y, right_z, right_roll, right_pitch, right_yaw, right_gripper,
                                         left_x, left_y, left_z, left_roll, left_pitch, left_yaw, left_gripper,
                                         chx, chy, chz, height, head_pit, head_yaw, mode1, mode2);

                        // 填充 ROS 消息
                        msgvrright.x = right_x;
                        msgvrright.y = right_y;
                        msgvrright.z = right_z;
                        msgvrright.roll = right_roll;
                        msgvrright.pitch = right_pitch;
                        msgvrright.yaw = right_yaw;
                        msgvrright.gripper = right_gripper;

                        msgvrleft.x = left_x;
                        msgvrleft.y = left_y;
                        msgvrleft.z = left_z;
                        msgvrleft.roll = left_roll;
                        msgvrleft.pitch = left_pitch;
                        msgvrleft.yaw = left_yaw;
                        msgvrleft.gripper = left_gripper;

                        msgvrleft.chx = chx;
                        msgvrleft.chy = chy;
                        msgvrleft.chz = chz;
                        msgvrleft.height = height;
                        msgvrleft.head_pit = head_pit;
                        msgvrleft.head_yaw = head_yaw;
                        msgvrleft.mode1 = mode1;
                        msgvrleft.mode2 = mode2;
                        VRRightpub.publish(msgvrright);
                        VRleftpub.publish(msgvrleft);
                }
                else
                {
                    ROS_WARN("Frame HEAD check failed: %02X %02X", buffer[0], buffer[1]);
                }
            }
        }
        catch (const std::exception &ex)
        {
            ROS_WARN("Error while receiving data: %s", ex.what());
        }
    }
    // 关闭串口
    sp.close();

    return 0;
}
