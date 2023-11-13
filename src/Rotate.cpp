//
// Created by Cassius0924 on 2020/03/03.
//

/*
 * SingleAzureKinect3DReconstruction
 * 此项目基于 Open3D 和 Azure Kinect DK 实现了三维重建。利用 Azure Kinect DK
 * 捕获图像并记录 IMU 数据，利用 Open3D 实现三维重建。
 */

#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <vector>

#include <k4a/k4a.hpp>
#include <open3d/Open3D.h>

#include "Bot.h"
#include "utility/Utility.h"

using namespace std;
using namespace open3d;
using namespace etrs::utility;

void onRotated(etrs::utility::Config &config, string &FIRST_MOTOR_ROTATION) {
    Debug::CoutSuccess("舵机旋转成功");
    if (FIRST_MOTOR_ROTATION == "F") {
        FIRST_MOTOR_ROTATION = "R";
        config.set("first_motor_rotation", "R");
    } else if (FIRST_MOTOR_ROTATION == "R") {
        FIRST_MOTOR_ROTATION = "F";
        config.set("first_motor_rotation", "F");
    } else {
        Debug::CoutError("未知的舵机旋转方向！");
    }
}

int main(int argc, char **argv) { // TODO: 可以传参，传入配置文件路径
    // 读取配置文件
    string config_file_path = "../default.conf";

    int an = 45;
    if (argc > 1) {
        // ans = argv[1];
        an = atoi(argv[1]);
    }
    etrs::utility::Config program_config(config_file_path);

    float INTERVAL_RADIAN = program_config.getFloat("interval_radian");
    float EXIT_RADIAN = program_config.getFloat("exit_radian");
    float VOXEL_SIZE = program_config.getFloat("voxel_size");
    float FINAL_VOXEL_SIZE = program_config.getFloat("final_voxel_size");
    float SMALL_RADIUS_MULTIPLIER = program_config.getFloat("small_radius_multiplier");
    float LARGE_RADIUS_MULTIPLIER = program_config.getFloat("large_radius_multiplier");
    bool IS_WRITE_FILE = program_config.getBool("is_write_file");
    string CLOUD_FILE_PATH = program_config.get("cloud_file_path");
    string MESH_FILE_PATH = program_config.get("mesh_file_path");
    bool IS_CREATE_SERVER = program_config.getBool("is_create_server");
    bool IS_CONNECT_ARM = program_config.getBool("is_connect_arm");
    bool IS_CONNECT_KINECT = program_config.getBool("is_connect_kinect");
    bool ENABLE_SOUND_SOURCE_LOCALIZATION = program_config.getBool("enable_sound_source_localization");
    int SERVER_PORT = program_config.getInt("protobuf_server_port");
    unsigned int SAMPLE_RATE = program_config.getInt("sample_rate");
    int SAMPLES = program_config.getInt("samples");
    int CHANNELS = program_config.getInt("channels");
    string MICROPHONE_NAME = program_config.get("microphone_name");
    string BOT_ARM_SERIAL_PORT_NAME = program_config.get("bot_arm_serial_prot_name");
    string BOT_CAR_SERIAL_PORT_NAME = program_config.get("bot_car_serial_prot_name");
    string STM32_SERIAL_PORT_NAME = program_config.get("stm32_serial_prot_name");
    string FIRST_MOTOR_ROTATION = program_config.get("first_motor_rotation");
    int MOTOR_SPEED = program_config.getInt("motor_speed");
    float TEMPERATURE_THRESHOLD = program_config.getFloat("temperature_threshold");
    float HUMIDITY_THRESHOLD = program_config.getFloat("humidity_threshold");
    string RECORDINGS_FOLDER_PATH = program_config.get("recordings_folder_path");
    float BLOCK_VOXEL_SIZE = program_config.getFloat("block_voxel_size");
    float TRUNC_VOXEL_MULTIPLIER = program_config.getFloat("trunc_voxel_multiplier");
    int BLOCK_RESOLUTION = program_config.getInt("block_resolution");
    int BLOCK_COUNT = program_config.getInt("block_count");
    float DEPTH_SCALE = program_config.getFloat("depth_scale");
    float DEPTH_MAX = program_config.getFloat("depth_max");
    float DEPTH_DIFF = program_config.getFloat("depth_diff");

    etrs::bot::BotMotor bot_motor(STM32_SERIAL_PORT_NAME);
    etrs::bot::BotCar bot_car(BOT_CAR_SERIAL_PORT_NAME, (char)0x12, 0.62);

    mutex stm32_mutex;

    // FIXME:
    bool need_reconnstrcution = true;
    int flag_recording = 0;
    bool kinect_going = true;

    thread receive_stm32_thread([&]() {
        unsigned char stm32_buffer[32];
        while (true) {
            unique_lock<mutex> lock(stm32_mutex);
            // 等待接受到stm32的数据
            bot_motor.recvData(stm32_buffer, 32);
            char data_type = stm32_buffer[0];
            // 解析stm32数据
            switch (data_type) {
                case 'M': {
                    if (stm32_buffer[4] == 'D' && stm32_buffer[5] == 'O' && stm32_buffer[6] == 'N' &&
                        stm32_buffer[7] == 'E') {
                        Debug::CoutDebug("舵机旋转完成");
                        flag_recording++;
                    }
                    break;
                }
            }
        }
    });

    char c;
    while (true) {
        cin >> c;
        if (c == 'w' || c == 'W') {
            std::cout << "向前移动" << std::endl;
            bot_car.moveForwardDistance(10);
        } else if (c == 'a' || c == 'A') {
            std::cout << "向左转动" << std::endl;
            bot_car.autoTurnByAngle(179);
        } else if (c == 's' || c == 'S') {
            std::cout << "向后移动" << std::endl;
            bot_car.moveBackwardDistance(10);
        } else if (c == 'd' || c == 'D') {
            std::cout << "向右转动" << std::endl;
            bot_car.autoTurnByAngle(-45);
        } else if (c == 'x' || c == 'X' || c == ' ') {
            std::cout << "停止移动" << std::endl;
            bot_car.stopCar();
        } else if (c == 'l') {
            bot_motor.rotate(-an, 3000);
        } else if (c == 'r') {
            bot_motor.rotate(an, 3000);
        }
    }
    return 1;
}
