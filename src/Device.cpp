#include "Device.h"

using namespace etrs::device;

Device::~Device() {}

Device::Device() {}

void Device::setDeviceName(string device_name) {
    this->device_name = device_name; // 不可以使用初始化列表: this->device_name(device_name)，因为这个函数不是构造函数
}
