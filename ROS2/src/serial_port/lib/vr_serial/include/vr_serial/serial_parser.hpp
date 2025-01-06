// serial_parser.hpp
#ifndef SERIAL_PARSER_HPP
#define SERIAL_PARSER_HPP

#include <vector>
#include <stdint.h>

struct ReceivePacket
{
  uint8_t header1 = 0x55;
  uint8_t header2 = 0xAA;
  uint8_t left_head = 0xF1; //左手帧头
  int left_XYZ[3];          //左手坐标
  int left_RPY[3];          //左手欧拉角
  int left_gripper;         //左手夹爪
  uint8_t right_head = 0xF2; //右手帧头
  int right_XYZ[3];          //右手坐标
  int right_RPY[3];          //右手欧拉角
  int right_gripper;         //右手夹爪
  uint8_t chassis_head = 0xF3; //底盘帧头
  int chassis_CHXYZ[3];     //底盘手柄坐标
  int chassis_PY[2];        //底盘头部欧拉角
  int height;               //底盘高度
  uint8_t mode1;            //底盘模式1
  uint8_t mode2;            //底盘模式2
  int time_count;           //数据包计数
  uint8_t end[3] = {0X0A,0x0D,0XEE}; //帧尾
} __attribute__((packed));


inline ReceivePacket fromBuffer(const uint8_t buffer[1024])
{
    ReceivePacket packet;
    // 直接拷贝 buffer 的前 94 个字节到 ReceivePacket 中
    std::copy(buffer, buffer + 94, reinterpret_cast<uint8_t*>(&packet));
    return packet;
}

ReceivePacket fromBuffer(const uint8_t* buffer);

void parseAndPublish(const uint8_t* buffer,
                     double& right_x, double& right_y, double& right_z,
                     double& right_roll, double& right_pitch, double& right_yaw, double& right_gripper,
                     double& left_x, double& left_y, double& left_z,
                     double& left_roll, double& left_pitch, double& left_yaw, double& left_gripper,
                     double& chx, double& chy, double& chz, double& height, double& head_pit, double& head_yaw,
                     uint8_t& mode1, uint8_t& mode2);

#endif // SERIAL_PARSER_HPP
