#ifndef _BOT_H_
#define _BOT_H_

#include "fcntl.h"
#include "pthread.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "termios.h"
#include "unistd.h"
#include <errno.h>
#include <string>
#include <time.h>

#include <functional>
#include <sys/select.h>
#include <sys/socket.h>

#include "Bluetooth.h"
#include "Serial.h"
#include "Utility.h"

using namespace std;
using namespace etrs::utility;

// 通信指令帧的定义
#define FLAG_START 0xfe
#define FLAG_END 0xfa

typedef unsigned char ComuType;
// 指令类型帧的定义
#define FREEMODE 0x13           // 自由模式
#define READANGLE 0x20          // 读取所有舵机的角度，不包括机械爪
#define POST_ALL_ANGLE 0x22     // 发送全部角度
#define READ_GRIPPER_ANGLE 0x65 // 读取夹爪角度
#define SET_GRIPPER_MODE 0x66   // 设置夹爪模式，张开和闭合两种模式
#define SET_GRIPPER_ANGLE 0x67  // 设置夹爪角度

#define READ_ALL_COORD 0x23 // 读取当前位置坐标
#define SET_ALL_COORD 0x25  // 发送位置坐标

#define DEFAULT_SERIAL_PORT_NAME "/dev/ttyUSB0"
#define WHEEL_D 20 // 单位厘米（cm）
#define WHEEL_DISTANCE 53

namespace etrs::bot {
// class BotArm {
// private:
//     int fd;
//     char command_buffer[20];
//     char gripper_buffer[7];
//     bool is_bluetooth;

// public:
//     enum CommandSet {
//         READ_ANGLE = 0x20,
//         READ_COORD = 0x23,
//         FREE_MODE = 0x1A,
//         SEND_ANGLE = 0x22,
//         SEND_COORD = 0x25,
//         TOGGLE_GRIPPER = 0x66,
//         SEND_GRIPPER_ANGLE = 0x67,
//     };
//     enum DataSet {
//         ALL_ANGLE = 0x20,
//         ALL_COORD = 0x23,
//     };
//     BotArm(string serial_port_name); // TODO: serial_port_name 改名
//     bool reset();
//     bool execute(const char *data_buffer, int length);
//     bool openGripper(const char speed);
//     bool closeGripper(const char speed);
//     int recvData(unsigned char *recv_buffer, const int recv_length);
//     bool sendCommand(CommandSet command_type);
// };

class BotArm {
private:
    Device *device;
    string port_or_address;
    char command_buffer[20];
    char gripper_buffer[7];
    bool is_bluetooth;

public:
    enum CommandSet {
        READ_ANGLE = 0x20,
        READ_COORD = 0x23,
        FREE_MODE = 0x1A,
        SEND_ANGLE = 0x22,
        SEND_COORD = 0x25,
        TOGGLE_GRIPPER = 0x66,
        SEND_GRIPPER_ANGLE = 0x67,
    };
    enum DataSet {
        ALL_ANGLE = 0x20,
        ALL_COORD = 0x23,
    };

public:
    BotArm(const string port_or_address, const string device_name = "###");
    void setDeviceName(const string device_name);
    bool reset();
    bool execute(const char *data_buffer, const int data_length);
    bool openGripper(const char speed);
    bool closeGripper(const char speed);
    int recvData(char *recv_buffer, const int recv_length);
    bool sendCommand(CommandSet command_type);
};

class STM32 {
protected:
    int fd;

public:
    enum CommandSet {
        MOTOR = 0x01,
        LED = 0x02,
    };
    STM32(string serial_port_name = DEFAULT_SERIAL_PORT_NAME);
    bool sendData(unsigned char *send_buffer, const int send_length);
    // bool sendData(char *send_buffer, const int send_length);
    int recvData(unsigned char *recv_buffer, const int recv_length);
    // void recvData(char *recv_buffer, const int recv_length);
};

// 虚继承，防止多重继承时出现多个fd
class BotMotor : virtual public STM32 {
private:
    unsigned char buffer[9];

public:
    BotMotor(string serial_port_name);
    bool rotate(string direction, std::function<void()> onRotated = nullptr);
    bool rotate(string direction, int angle, int speed, std::function<void()> onRotated = nullptr);
    bool rotate(int angle, int speed, std::function<void()> onRotated = nullptr);
};

class BotCar {
private:
    int fd;
    float speed;
    float angle_speed;
    unsigned char buffer[10];
    unsigned char buffer2[10];

public:
    BotCar(string serial_port_name, const char speed_value, float scale = 1.0f);
    bool sendData(unsigned char *send_buffer, const int send_length);
    int recvData(unsigned char *recv_buffer, const int recv_length);
    void setSpeed(const char speed_value, float scale = 1.0);
    bool moveForward();
    bool moveForwardTime(float time);
    bool moveForwardDistance(float distance);
    bool moveBackward();
    bool moveBackwardTime(float time);
    bool moveBackwardDistance(float distance);
    bool turnLeft();
    bool turnLeftTime(float time);
    bool turnLeftAngle(float angle);
    bool turnRight();
    bool turnRightTime(float time);
    bool turnRightAngle(float angle);
    bool turnAngle(float anlge);
    bool autoTurnByAngle(float angle);
    bool autoTurnByAngleAndSpeed(float angle, char left_speed_value, char right_speed_value);
    bool stopCar();
    bool executeMoveSequence(float *seq, int seq_length);
};

class BotLed : virtual public STM32 {
private:
    unsigned char buffer[8];

public:
    enum LedColor {
        RED = 0x01,
        GREEN = 0x02,
        BLUE = 0x03,
        CUSTOM = 0x04,
    };
    BotLed(string serial_port_name);
    bool setLedColor(LedColor color);
    bool setLedColor(int r, int g, int b);
};
} // namespace etrs::bot

#endif //_BOT_H_