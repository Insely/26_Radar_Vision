#include "serial/serial.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>

SerialPort::SerialPort(const std::string &port_name, int baudrate) 
    : port_name_(port_name), baudrate_(baudrate), fd_(-1) {}

SerialPort::~SerialPort() {
    closePort();
}

bool SerialPort::init() {
    fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        perror("无法打开串口");
        return false;
    }

    struct termios options;
    tcgetattr(fd_, &options);

    speed_t speed = B115200;
    if (baudrate_ == 921600) speed = B921600;
    else if (baudrate_ == 460800) speed = B460800;
    
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

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

    const char *buf = reinterpret_cast<const char *>(&data);
    size_t total = sizeof(VisionData);
    size_t written = 0;

    while (written < total) {
        ssize_t n = write(fd_, buf + written, total - written);
        if (n <= 0) {
            closePort();
            if (!init()) {
                std::cerr << "串口重连失败，放弃发送" << std::endl;
                return false;
            }
            written = 0;
            continue;
        }
        written += n;
    }
    return true;
}

void SerialPort::closePort() {
    if (fd_ != -1) {
        close(fd_);
        fd_ = -1;
    }
}
