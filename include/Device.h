#ifndef _DEVICE_H_
#define _DEVICE_H_

#include <iostream>

using namespace std;

namespace etrs::device {
    class Device {
    public:
        int fd;
        string device_name;

    public:
        void setDeviceName(const string device_name);
        // virtual关键字用于指定该函数为虚函数,也就是说该函数可以被子类重写(override)。如果不加virtual,即使子类重写了该函数,也会调用父类的函数版本。
        // =0表示该函数为纯虚函数,意味着该函数必须被子类实现、重写,不能直接调用。纯虚函数通常用于接口类中。
        virtual int sendData(const char *data_buffer, const int data_length) = 0;
        virtual int recvData(char *recv_buffer, const int recv_length) = 0;
    };
} // namespace etrs::device

#endif // _DEVICE_H_