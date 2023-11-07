#include "Bot.h"

#include <cmath>
#include <iostream>
#include <thread>

using namespace std;
using namespace etrs::bot;

void rightArmAutoRealy(unsigned char *databuff) {
    int i = 0;
    int new_angle = 0, temp = 0; // 解析到原始的角度在取反后
    if (databuff == NULL) {
        return;
    }
    for (i = 1; i <= 5; i++) {                                            // 只处理1-5号机位
        temp = databuff[(i - 1) * 2] * 0x100 + databuff[(i - 1) * 2 + 1]; // 0x100 = 256
        new_angle = ((temp > 33000 ? (temp - 65536) : temp) / 100) * -1;  // 新角度值

        databuff[(i - 1) * 2] = (unsigned char)((((new_angle * 100) + 65536) % 65536) / 0x100); // 新角度数据值的高字节
        databuff[(i - 1) * 2 + 1] =
            (unsigned char)((((new_angle * 100) + 65536) % 65536) % 0x100); // 新角度数据值的低字节
    }
    return;
}

BotArm::~BotArm() {
    if (this->device != nullptr) {
        delete this->device;
    }
}

BotArm::BotArm(const string port_or_address, const string device_name) : port_or_address(port_or_address) {
    bool is_mac_address = MacAddress::isValidMacAddress(port_or_address);
    if (is_mac_address) {         // 如果是MAC地址
        char op_code[1] = {0x52}; // TODO: 目前写死，需要修改成可变的
        char handle[2] = {0x2d, 0x00};
        this->device = new etrs::device::bt::BleDevice(port_or_address, op_code, handle, device_name);
    } else { // 否则是串口地址
        this->device = new etrs::device::serial::SerialDevice(port_or_address, device_name);
    }
    // 机械臂关节舵机默认速度 30%
    this->arm_speed = 0x1E;

    // this->command_buffer[0] = 0xFE;
    // this->command_buffer[1] = 0xFE;

    this->gripper_buffer[0] = 0xFE;
    this->gripper_buffer[1] = 0xFE;
    this->gripper_buffer[2] = 0x04;
    this->gripper_buffer[3] = CommandTypeSet::SEND_GRIPPER_ANGLE;

    this->gripper_buffer[5] = 0x14;
    this->gripper_buffer[6] = 0xFA;
}

void BotArm::setDeviceName(const string device_name) { this->device->setDeviceName(device_name); }

void BotArm::setArmSpeed(const char arm_speed) { this->arm_speed = arm_speed; }

char BotArm::getArmSpeed() { return this->arm_speed; }

int BotArm::anglesToCommand(const int *angles, char *&command) {
    if (angles == nullptr) {
        Debug::CoutError("机械臂角度指令转换失败！角度指针或指令指针为空！");
        return 0;
    }
    // 指令例子： 0xFE, 0xFE, 0x0F, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E,
    int cmd_len = initCommand(CommandTypeSet::SEND_ALL_ANGLE, command);
    for (int i = 0; i < 6; i++) {
        int angle = angles[i] * 100;
        if (angle < 0) {
            angle = angle + 65536;
        }
        command[4 + 2 * i] = (angle >> 8) & 0xFF; // 角度高位
        command[5 + 2 * i] = angle & 0xFF;        // 角度低位
    }
    command[16] = angles[6];

    // 返回指令长度
    return cmd_len;
}

int BotArm::coordToCommand(const float *coord, char *&command) {
    // TODO: 末端坐标模式
    return 0;
}

bool BotArm::reset() {
    // char reset_buffer[18] = {0xFE, 0xFE, 0x0F, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00,
    //                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0xFA};
    int reset_angles[7] = {0, 0, 0, 0, 0, 0, 30};
    return executeByAngle(reset_angles);
}

bool BotArm::executeByAngle(const int *angles) {
    char *data_buffer = nullptr;
    int data_length = anglesToCommand(angles, data_buffer);
    // char reset_buffer[18] = {0xFE, 0xFE, 0x0F, 0x22, 0x00, 0x00, 0x00, 0x00, 0x00,
    //                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0xFA};
    // cout << "RESET: ";
    // for (int i = 0; i < 18; i++) {
    //     cout << hex << (int)reset_buffer[i] << ",";
    // }
    // cout << endl;
    // cout << "DATA: ";
    // for (int i = 0; i < 18; i++) {
    //     cout << hex << (int)data_buffer[i] << ",";
    // }
    // cout << endl;
    return this->device->sendData(data_buffer, data_length) > 0;
}

bool BotArm::executeByCoord(const float *coord) {
    // TODO: 末端坐标模式
    return false;
}

int BotArm::recvData(char *recv_buffer, const int recv_length) {
    return this->device->recvData(recv_buffer, recv_length);
}

int BotArm::initCommand(BotArm::CommandTypeSet command_type, char *&command) {
    int data_len;
    switch (command_type) {
        case BotArm::CommandTypeSet::READ_ANGLE: {
            data_len = 2;
            break;
        }
        case BotArm::CommandTypeSet::READ_COORD: {
            data_len = 2;
            break;
        }
        case BotArm::CommandTypeSet::FREE_MODE: {
            data_len = 2;
            break;
        }
        case BotArm::CommandTypeSet::SEND_ALL_ANGLE: {
            data_len = 15;
            break;
        }
        default:
            Debug::CoutError("未定义的机械臂指令类型！");
            // command = new char[1];
            // command[0] = 0x01;
            return 0;
    }
    command = new char[data_len + 3]; // 指令长度 = 数据帧长度 + 2个识别帧 + 1个结束帧
    command[0] = 0xFE;                // 识别帧
    command[1] = 0xFE;                // 识别帧
    command[2] = (char)data_len;      // 数据帧长度
    command[3] = command_type;        // 指令帧
    command[data_len + 2] = 0xFA;     // 结束帧
    return data_len + 3;
}

bool BotArm::sendCommand(BotArm::CommandTypeSet command_type) {
    // char *command;
    // int cmd_len = initCommand(command_type, command);
    // memcpy(this->command_buffer + 2, command, command_length);
    // this->command_buffer[command_length + 2] = 0xFA; // 结束帧

    // // 计算方法：角度值低位 + 角度高位值乘以256 先判断是否大于33000 如果大于33000就再减去65536 最后除以100
    // // 如果小于33000就直接除以100
    // //  0x01 0x20 0xFA
    // return this->device->sendData(this->command_buffer, cmd_len + 3);
    return false;
}

bool BotArm::openGripper(const char speed) {
    this->gripper_buffer[4] = 0x64;
    this->gripper_buffer[5] = speed;
    return this->device->sendData(this->gripper_buffer, 7);
}

bool BotArm::closeGripper(const char speed) {
    this->gripper_buffer[4] = 0x00;
    this->gripper_buffer[5] = speed;
    return this->device->sendData(this->gripper_buffer, 7);
}

STM32::~STM32() {}

STM32::STM32(string serial_port_name) {
    struct termios newtio;
    tcgetattr(this->fd, &newtio);
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
    this->fd = open(serial_port_name.c_str(), O_RDWR); // 读写方式打开串口

    if (this->fd < 0) {
        Debug::CoutError("STM32连接失败！stm32_fd = {}", this->fd);
        return;
    }
    Debug::CoutSuccess("STM32连接成功！stm32_fd = {}", this->fd);

    if (tcsetattr(this->fd, TCSADRAIN, &newtio) != 0) {
        Debug::CoutError("串口初始化失败！");
        return;
    }
    Debug::CoutSuccess("STM32设备初始化成功！");
}

bool STM32::sendData(unsigned char *send_buffer, const int send_length) {
    if (write(this->fd, send_buffer, send_length) < 0) {
        return false;
    }
    return true;
}

int STM32::recvData(unsigned char *recv_buffer, const int recv_length) {
    int len = -1;
    while ((len = read(this->fd, recv_buffer, recv_length)) < 0)
        ;
    return len;
}

BotMotor::~BotMotor() {}

BotMotor::BotMotor(string serial_port_name = DEFAULT_SERIAL_PORT_NAME) : STM32(serial_port_name) {
    this->buffer[0] = 0xFF; // 包头
    this->buffer[1] = BotMotor::CommandTypeSet::MOTOR;
    this->buffer[2] = 0x00; // 方向
    this->buffer[3] = 0x00; // 角度高位
    this->buffer[4] = 0x00; // 角度低位
    this->buffer[5] = 0x00; // 速度高位
    this->buffer[6] = 0x00; // 速度低位
    this->buffer[7] = 0xFE; // 包尾
}

bool BotMotor::rotate(string direction, function<void()> onRotated) {
    return rotate(direction, 360, 10000, onRotated);
    //     if (direction == "F") {
    //     this->buffer[2] = 0x01; // 顺时针
    //     this->buffer[3] = 0x01;
    //     this->buffer[4] = 0x68;
    //     this->buffer[5] = 0x27;
    //     this->buffer[6] = 0x10;
    //     Debug::CoutDebug("向前转动电机！");
    //     return STM32::sendData(this->buffer, 8);
    // } else if (direction == "R") {
    //     this->buffer[2] = 0x00;
    //     this->buffer[3] = 0x01;
    //     this->buffer[4] = 0x68;
    //     this->buffer[5] = 0x27;
    //     this->buffer[6] = 0x10;
    // }
}

bool BotMotor::rotate(string direction, int angle, int speed, function<void()> onRotated) {
    if (onRotated != nullptr) {
        onRotated();
    }
    if (direction == "F") {
        this->buffer[2] = 0x01; // 顺时针
        this->buffer[3] = (angle >> 8) & 0xFF;
        this->buffer[4] = angle & 0xFF;
        this->buffer[5] = (speed >> 8) & 0xFF;
        this->buffer[6] = speed & 0xFF;
        return STM32::sendData(this->buffer, 8);
    } else if (direction == "R") {
        this->buffer[2] = 0x00;
        this->buffer[3] = (angle >> 8) & 0xFF;
        this->buffer[4] = angle & 0xFF;
        this->buffer[5] = (speed >> 8) & 0xFF;
        this->buffer[6] = speed & 0xFF;
        return STM32::sendData(this->buffer, 8);
    }
    Debug::CoutDebug("失败！{}", direction);
    return false;
}

bool BotMotor::rotate(int angle, int speed, function<void()> onRotated) {
    if (onRotated != nullptr) {
        onRotated();
    }
    this->buffer[2] = angle > 0 ? 0x01 : 0x00; // 顺时针
    angle = abs(angle);
    this->buffer[3] = (angle >> 8) & 0xFF;
    this->buffer[4] = angle & 0xFF;
    this->buffer[5] = (speed >> 8) & 0xFF;
    this->buffer[6] = speed & 0xFF;
    return STM32::sendData(this->buffer, 8);
}


BotCar::~BotCar(){}

BotCar::BotCar(string serial_port_name, const char speed_value, float scale) {
    struct termios newtio;
    tcgetattr(this->fd, &newtio);
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag = B115200;
    newtio.c_cflag |= CS8;
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY);
    newtio.c_oflag &= ~OPOST;
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VTIME] = 0;
    this->fd = open(serial_port_name.c_str(), O_RDWR);

    if (this->fd < 0) {
        Debug::CoutError("底盘车设备连接失败！bot_car_fd = {}", this->fd);
        Debug::CoutError("串口打开失败！");
        return;
    }
    Debug::CoutSuccess("底盘车设备连接成功！bot_car_fd = {}", this->fd);

    if (tcsetattr(this->fd, TCSADRAIN, &newtio) != 0) {
        Debug::CoutError("串口初始化失败！");
        return;
    }
    // Debug::CoutSuccess("底盘车设备初始化成功！", this->fd);
    setSpeed(speed_value, scale);
}

bool BotCar::sendData(unsigned char *send_buffer, const int send_length) {
    if (write(this->fd, send_buffer, send_length) < 0) {
        return false;
    }
    return true;
}

int BotCar::recvData(unsigned char *recv_buffer, const int recv_length) {
    int len = -1;
    while ((len = read(this->fd, recv_buffer, recv_length)) < 0)
        ;
    return len;
}

/**
 * 假如 A 电机的速度为 0x12，即十进制18。这样相当于控制A电机以18的速度运动。下面解释一下这个 18 的单位：
 * 10ms 转 18(0x12) 个脉冲，1s 转 1800 个脉冲。车轮转一圈，输出(编码器线数 500 * 减速比 28 * 倍频 4) = 56000 个脉冲,
 * 实际上因为光电编码器精度非常高，为了方便信号处理与传输，在代码里面已经除以了 8 的，所以相当于车轮转一圈输出 7000
 * 个脉冲。 也就是每秒转 1800/7000 圈，即 0.2571 圈，再结合轮胎的直径信息，就可以得到小车的运行速度。
 * 已知小车车轮的直径为 20cm，即周长为 62.8cm。
 * 小车的速度可以通过每秒转的圈数乘以轮胎的周长来计算，即 0.2571 圈/秒 * 62.8 厘米/圈 ≈ 16.12 厘米/秒。
 * 旋转的角速度可以通过两个轮子的速度差除以两个轮子的轴距来计算。
 * 已知小车轮距为 53cm，即旋转角速度 = abs(left_speed - right_speed) / 53。
 */
void BotCar::setSpeed(const char speed_value, float scale) {
    this->buffer[0] = 0xff;
    this->buffer[1] = 0xfe;
    this->buffer[2] = speed_value;
    this->buffer[3] = speed_value;
    this->buffer[4] = 0x01;
    this->buffer[5] = 0x01;
    this->buffer[6] = 0x00;
    this->buffer[7] = 0x00;
    this->buffer[8] = 0x00;
    this->buffer[9] = 0x00;

    this->buffer2[0] = 0xff;
    this->buffer2[1] = 0xfe;
    this->buffer2[2] = speed_value; // A速度
    this->buffer2[3] = speed_value; // B速度
    this->buffer2[4] = 0x00;        // 方向，1左转，2右转
    this->buffer2[5] = 0x00;        // 角度
    this->buffer2[6] = 0x01;        // 模式：1角度旋转模式
    this->buffer2[7] = 0x00;
    this->buffer2[8] = 0x00;
    this->buffer2[9] = 0x00;

    // 计算小车移动线速度
    this->speed = ((int)speed_value) * 100 / 7000.0 * WHEEL_D * M_PI * scale;
    Debug::CoutInfo("小车移动线速度（cm/s）：{}", this->speed);
    // 计算小车旋转角速度
    this->angle_speed = 2 * (int)speed_value * 100 / 7000.0 * WHEEL_D * M_PI / WHEEL_DISTANCE * 180.0 / M_PI;
    Debug::CoutInfo("小车旋转角速度（deg/s）：{}", this->angle_speed);
}

bool BotCar::moveForward() {
    this->buffer[4] = 0x01;
    this->buffer[5] = 0x01;
    return sendData(this->buffer, 10);
}
bool BotCar::moveForwardTime(float time_s) {
    if (!moveForward()) {
        return false;
    }
    this_thread::sleep_for(chrono::milliseconds((int)(time_s * 1000)));
    return stopCar();
}
bool BotCar::moveForwardDistance(float distance) {
    if (distance < 0) {
        return false;
    } else if (distance == 0) {
        return true;
    }
    return moveForwardTime(distance / this->speed);
}

bool BotCar::moveBackward() {
    this->buffer[4] = 0x00;
    this->buffer[5] = 0x00;
    return sendData(this->buffer, 10);
}
bool BotCar::moveBackwardTime(float time_s) {
    if (!moveBackward()) {
        return false;
    }
    this_thread::sleep_for(chrono::milliseconds((int)(time_s * 1000)));
    return stopCar();
}
bool BotCar::moveBackwardDistance(float distance) {
    if (distance < 0) {
        return false;
    } else if (distance == 0) {
        return true;
    }
    return moveBackwardTime(distance / this->speed);
}

bool BotCar::turnLeft() {
    this->buffer[4] = 0x00;
    this->buffer[5] = 0x01;
    return sendData(this->buffer, 10);
}
bool BotCar::turnLeftTime(float time_s) {
    if (!turnLeft()) {
        return false;
    }
    this_thread::sleep_for(chrono::milliseconds((int)(time_s * 1000)));
    return stopCar();
}
bool BotCar::turnLeftAngle(float angle) {
    if (angle <= 0) {
        return false;
    }
    return turnLeftTime(angle / this->angle_speed);
}

bool BotCar::turnRight() {
    this->buffer[4] = 0x01;
    this->buffer[5] = 0x00;
    return sendData(this->buffer, 10);
}

bool BotCar::turnRightTime(float time_s) {
    if (!turnRight()) {
        return false;
    }
    this_thread::sleep_for(chrono::milliseconds((int)(time_s * 1000)));
    return stopCar();
}
bool BotCar::turnRightAngle(float angle) {
    if (angle <= 0) {
        return false;
    }
    return turnRightTime(angle / this->angle_speed);
}

bool BotCar::turnAngle(float angle) {
    if (angle < 0) {
        return turnLeftAngle(-angle);
    } else if (angle > 0) {
        return turnRightAngle(angle);
    }
    return true;
}

bool BotCar::autoTurnByAngle(float angle) {
    if (angle > 0) {
        this->buffer2[4] = 0x01; // 大于0左转，小于0右转
        this->buffer2[5] = (int)angle - 2;
    } else if (angle < 0) {
        this->buffer2[4] = 0x02; // 大于0左转，小于0右转
        this->buffer2[5] = -(int)angle - 2;
    } else {
        return false;
    }
    return sendData(this->buffer2, 10);
}

bool BotCar::autoTurnByAngleAndSpeed(float angle, char left_speed_value, char right_speed_value) {
    int left_temp = this->buffer2[2];
    int right_temp = this->buffer2[3];
    this->buffer2[2] = left_speed_value;
    this->buffer2[3] = right_speed_value;
    if (!autoTurnByAngle(angle)) {
        return false;
    }
    this->buffer2[2] = left_temp;
    this->buffer2[3] = right_temp;
    return true;
}

bool BotCar::stopCar() {
    int left_temp = this->buffer[2];
    int right_temp = this->buffer[3];
    this->buffer[2] = 0x00;
    this->buffer[3] = 0x00;
    if (!sendData(this->buffer, 10)) {
        return false;
    }
    this->buffer[2] = left_temp;
    this->buffer[3] = right_temp;
    return true;
}

bool BotCar::executeMoveSequence(float *seq, int seq_length) {
    float flag = seq[0];
    if (flag == 1) { // 先移动
        // 奇数下标为移动，偶数下标为旋转
        for (int i = 1; i < seq_length; i++) {
            Debug::CoutDebug("执行：{}", seq[i]);
            if (i % 2 == 1) { // 如果是奇数
                moveForwardDistance(seq[i]);
                this_thread::sleep_for(chrono::milliseconds(500));
            } else {
                // turnAngle(-seq[i]);

                // FIXME: 817比赛临时关闭旋转
                // autoTurnByAngle(seq[i]);
                // this_thread::sleep_for(chrono::milliseconds(8000));
                this_thread::sleep_for(chrono::milliseconds(2000));
            }
        }
    } else if (flag == 0) { // 先旋转
        // 奇数下标为旋转，偶数下标为移动
        for (int i = 1; i < seq_length; i++) {
            Debug::CoutDebug("执行：{}", seq[i]);
            if (i % 2 == 1) { // 如果是奇数
                // turnAngle(-seq[i]);

                // FIXME: 817比赛临时关闭旋转
                // autoTurnByAngle(seq[i]);
                // this_thread::sleep_for(chrono::milliseconds(8000));
                this_thread::sleep_for(chrono::milliseconds(2000));
            } else {
                moveForwardDistance(seq[i]);
                this_thread::sleep_for(chrono::milliseconds(500));
            }
        }
    } else {
        Debug::CoutError("未知的底盘车移动序列");
        return false;
    }
    return true;
}


BotLed::~BotLed(){}

BotLed::BotLed(string serial_port_name) {
    this->buffer[0] = 0xFF;
    this->buffer[1] = BotLed::CommandTypeSet::LED;
    this->buffer[2] = 0x00; // 颜色
    this->buffer[3] = 0x00; // r
    this->buffer[4] = 0x00; // g
    this->buffer[5] = 0x00; // b
    this->buffer[6] = 0x00;
    this->buffer[7] = 0xFE;
}
bool BotLed::setLedColor(LedColor color) {
    this->buffer[2] = color;
    return STM32::sendData(this->buffer, 8);
}

bool BotLed::setLedColor(int r, int g, int b) {
    this->buffer[2] = LedColor::CUSTOM;
    this->buffer[3] = r;
    this->buffer[4] = g;
    this->buffer[5] = b;
    return STM32::sendData(this->buffer, 8);
}