#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <Device.h>
#include <Utility.h>

#include "fcntl.h"
#include "pthread.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "termios.h"
#include "unistd.h"
#include <errno.h>
#include <iostream>
#include <sys/select.h>
#include <sys/socket.h>

using namespace std;
using namespace etrs::device;
using namespace etrs::utility;

namespace etrs::device::serial {
    class SerialDevice : public Device { // 继承Device基类
    private:
        string serial_port_name;

    public:
        explicit SerialDevice(const string serial_port_name, const string device_name = "###");
        int sendData(const char *data_buffer, const int data_length) override;
        int recvData(char *recv_buffer, const int recv_length) override;

    private:
        int serialConnect(string serial_port_name);
    };
} // namespace etrs::device::serial

#endif // _SERIAL_H_