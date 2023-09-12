#include <Bluetooth.h>

using namespace etrs::device::bt;

// 通过MAC地址连接蓝牙设备
// 通过 MAC 地址连接蓝牙设备，并创建创建 socket，返回 socket 文件描述符
#define ATT_CID 4

BleDevice::BleDevice(string mac_address, string device_name) : mac_address(mac_address) {
    setDeviceName(device_name);
    this->fd = bleConnectL2cap(mac_address);
}

BleDevice::BleDevice(string mac_address, const char *op_code, const char *handle, string device_name)
    : mac_address(mac_address) {
    setDeviceName(device_name);
    strcpy(this->op_code, op_code);
    strcpy(this->handle, handle);
    this->fd = bleConnectL2cap(mac_address);
}

int BleDevice::bleConnectL2cap(string mac_address) { // arm mac address: 08:B6:1F:C1:DB:1A
    int fd = socket(PF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
    if (fd < 0) {
        Debug::CoutError("{}: {}，创建L2CAP socket失败", this->device_name, mac_address);
        return -1;
    }

    struct sockaddr_l2 bind_addr = {0};
    bind_addr.l2_family = AF_BLUETOOTH;
    bind_addr.l2_cid = htobs(ATT_CID); // ATT CID
    bdaddr_t any_addr = {{0, 0, 0, 0, 0, 0}};
    bacpy(&bind_addr.l2_bdaddr, &any_addr);
    bind_addr.l2_bdaddr_type = BDADDR_LE_PUBLIC;

    int err = bind(fd, (struct sockaddr *)&bind_addr, sizeof(bind_addr));
    if (err) {
        Debug::CoutError("{}: {}，绑定L2CAP socket失败", this->device_name, mac_address);
        return -1;
    }

    struct sockaddr_l2 conn_addr = {0};
    conn_addr.l2_family = AF_BLUETOOTH;
    conn_addr.l2_cid = htobs(ATT_CID); // ATT CID
    str2ba(mac_address.c_str(), &conn_addr.l2_bdaddr);
    conn_addr.l2_bdaddr_type = BDADDR_LE_PUBLIC;

    err = connect(fd, (struct sockaddr *)&conn_addr, sizeof(conn_addr));
    if (err) {
        Debug::CoutError("{}: {}，连接L2CAP socket失败", this->device_name, mac_address);
        return -1;
    }
    Debug::CoutSuccess("{}设备连接成功！fd = {}", this->device_name, fd);

    // MTU默认23字节： op code(1 字节)，handle(2 字节，小端)，payload(0-20字节)
    // char on[] = {0x12, 0x2d, 0x00, 0xFE, 0xFE, 0x0F, 0x22, 0x00, 0x00, 0x00, 0x00,
    //              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0xFA};
    // char op[] = {0x12};
    // char ha[] = {0x2d, 0x00};
    // char on[18] = {0xFE, 0xFE, 0x0F, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00,
    //              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0xFA};
    // Debug::CoutDebug("on: {}", sizeof(on));
    // ble_send_l2cap(fd, op, ha, on, sizeof(on));

    return fd;
}

void BleDevice::setOpCode(const char *op_code) { strcpy(this->op_code, op_code); }

void BleDevice::setHandle(const char *handle) { strcpy(this->handle, handle); }

void BleDevice::setOpCodeAndHandle(const char *op_code, const char *handle) {
    strcpy(this->op_code, op_code);
    strcpy(this->handle, handle);
}

// MTU默认23字节： op code(1 字节)，handle(2 字节，小端)，payload(0-20字节)
int BleDevice::sendData(const char *op_code, const char *handle, const char *data_buffer, const int data_length) {
    int len = -1;
    unsigned char buf[512];
    memcpy(buf, op_code, 1);
    memcpy(buf + 1, handle, 2);
    memcpy(buf + 3, data_buffer, data_length);
    if (len = write(this->fd, buf, data_length + 3) < 0) {
        Debug::CoutError("发送数据失败");
        return -1;
    }
    return len;
}

int BleDevice::sendData(const char *data_buffer, const int data_length) {
    // 将 op code, handle, data 拼接成一个数组，使用拼接函数
    int len = -1;
    unsigned char buf[512];
    memcpy(buf, this->op_code, 1);
    memcpy(buf + 1, this->handle, 2);
    memcpy(buf + 3, data_buffer, data_length);
    if (len = write(this->fd, buf, data_length + 3) < 0) {
        Debug::CoutError("发送数据失败");
        return -1;
    }
    return len;
}

int BleDevice::recvData(char *recv_buffer, const int recv_length) {
    int len = -1;
    while ((len = read(this->fd, recv_buffer, recv_length)) < 0)
        ;
    return len;
}

int BleDevice::modifyMtu(const int mtu) {
    // FIXME: 未完成，以下代码无法修改MTU
    unsigned char buf[23] = {0x02, 0x0A};
    int len = -1;
    if (len = write(this->fd, buf, sizeof(buf)) < 0) {
        Debug::CoutError("修改MTU失败");
        return -1;
    }
    return len;
}
