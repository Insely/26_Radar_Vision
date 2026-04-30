#include "serial/serial.hpp"
#include <fcntl.h>      // 文件控制定义
#include <unistd.h>     // UNIX 标准函数定义
#include <termios.h>    // POSIX 终端控制定义
#include <cstring>

SerialPort::SerialPort(const std::string &port_name, int baudrate) 
    : port_name_(port_name), baudrate_(baudrate), fd_(-1) {}

SerialPort::~SerialPort() {
    closePort();
}

bool SerialPort::init() {
    // 1. 打开串口设备
    fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        perror("无法打开串口");
        return false;
    }

    // 2. 配置串口参数
    struct termios options;
    tcgetattr(fd_, &options);

    // 设置波特率 (默认 115200)
    speed_t speed = B115200;
    if (baudrate_ == 921600) speed = B921600;
    else if (baudrate_ == 460800) speed = B460800;
    
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 设置数据位 (8位), 无奇偶校验, 1位停止位
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // 设置为原始模式 (Raw Mode)，不进行任何字符处理
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

    // 刷新并设置
    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
        perror("串口配置失败");
        return false;
    }

    std::cout << "串口初始化成功: " << port_name_ << " 波特率: " << baudrate_ << std::endl;
    return true;
}

bool SerialPort::send(VisionData &data) {
    if (fd_ == -1) return false;

    // 直接将结构体作为字节流写入串口
    int bytes_sent = write(fd_, &data, sizeof(VisionData));
    
    if (bytes_sent == -1) {
        // 如果发送失败，尝试重新初始化（参考原有异常处理）
        closePort();
        init();
        return false;
    }
    return true;
}

void SerialPort::closePort() {
    if (fd_ != -1) {
        close(fd_);
        fd_ = -1;
    }
}