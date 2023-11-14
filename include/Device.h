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
        Device();

        virtual ~Device();

        void setDeviceName(const string device_name);

        virtual int sendData(const char *data_buffer, const int data_length) = 0;

        virtual int recvData(char *recv_buffer, const int recv_length) = 0;
    };
} // namespace etrs::device

#endif // _DEVICE_H_