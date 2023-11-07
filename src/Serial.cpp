#include "Serial.h"

using namespace etrs::device::serial;

SerialDevice::~SerialDevice() {}

SerialDevice::SerialDevice(const string serial_port_name, const string device_name)
    : serial_port_name(serial_port_name) {
    this->fd = serialConnect(serial_port_name);
    setDeviceName(device_name);
}

int SerialDevice::serialConnect(string serial_port_name) {
    struct termios newtio;
    tcgetattr(fd, &newtio);
    newtio.c_cflag &= ~CSIZE;         // 数据位屏蔽 将c_cflag全部清零
    newtio.c_cflag = B115200;         // set bound
    newtio.c_cflag |= CS8;            // 数据位8
    newtio.c_cflag |= CLOCAL | CREAD; // 使驱动程序启动接收字符装置，同时忽略串口信号线的状态
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控制
    newtio.c_oflag &= ~OPOST; // 使用原始输出，就是禁用输出处理，使数据能不经过处理、过滤地完整地输出到串口接口。
    newtio.c_lflag &=
        ~(ICANON | ECHO | ECHOE | ISIG); // 在原始模式下，串口输入数据是不经过处理的，在串口接口接收的数据被完整保留。
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 0;
    int fd = open(serial_port_name.c_str(), O_RDWR); // 读写方式打开串口
    if (fd < 0) {
        Debug::CoutError("{}设备连接失败！", this->device_name, fd);
        return -1;
    }
    Debug::CoutSuccess("{}设备连接成功！fd = {}", this->device_name, fd);

    if (tcsetattr(fd, TCSADRAIN, &newtio) != 0) {
        Debug::CoutError("串口初始化失败！");
        return -1;
    }
    Debug::CoutSuccess("{}设备初始化成功！", this->device_name);
    return fd;
}

int SerialDevice::sendData(const char *data_buffer, const int data_length) {
    int len = -1;
    if ((len = write(this->fd, data_buffer, data_length)) < 0) {
        return -1;
    }
    return len;
}

int SerialDevice::recvData(char *recv_buffer, const int recv_length) {
    int len = -1;
    if ((len = read(this->fd, recv_buffer, recv_length)) < 0) {
        return -1;
    }
    return len;
}
