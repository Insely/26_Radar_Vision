#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <iostream>
#include <string>
#include <stdint.h>

// 串口通信数据结构体
struct VisionData {
    uint8_t  header = 0xA5;    // 1. 帧头校验 0xA5
    float    yaw_error;        // 2. 水平偏差 (float, 4字节)
    uint8_t  at_center;        // 3. 是否到达水平中心 (0/1)
    uint8_t  allow_fire;       // 4. 是否允许发射 (0/1)
} __attribute__((packed));     // 确保字节对齐

class SerialPort {
public:
    // 构造函数，需要传入串口设备名（如 "/dev/ttyUSB0"）和波特率
    SerialPort(const std::string &port_name, int baudrate);
    ~SerialPort();

    // 初始化串口
    bool init();
    // 发送数据包
    bool send(VisionData &data);
    // 关闭串口
    void closePort();

private:
    int fd_;                   // 串口文件描述符
    std::string port_name_;    // 串口设备名
    int baudrate_;             // 波特率
};

#endif