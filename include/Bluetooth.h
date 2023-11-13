#ifndef _BLUETOOTH_H_
#define _BLUETOOTH_H_

// TODO: 删除无用的include
#include <sys/uio.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <iostream>
#include <vector>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/rfcomm.h>

#include "Device.h"
#include "utility/Utility.h"

using namespace etrs::utility;
using namespace std;
using namespace etrs::device;

namespace etrs::device::bt {
    class BleDevice : public Device { // 继承Device基类
    private:
        string mac_address;
        char op_code[1];
        char handle[2];

    public:
        explicit BleDevice(string mac_address, string device_name = "###");
        explicit BleDevice(string mac_address, const char *op_code, const char *handle, string device_name = "###");
        ~BleDevice();

        void setOpCode(const char *op_code);
        void setHandle(const char *handle);
        void setOpCodeAndHandle(const char *op_code, const char *handle); // TODO: 名字太长

        int sendData(const char *data_buffer, const int data_length); // 基于 l2cap 协议

        int sendData(const char *op_code, const char *handle, const char *data_buffer, const int data_length);

        int recvData(char *recv_buffer, const int recv_length) override;

        int modifyMtu(const int mtu);

    private:
        int bleConnectL2cap(string mac_address);
    };
} // namespace etrs::device::bt

#endif // _BLUETOOTH_H_