#include "AzureKinect.h"
#include "AzureKinectExtrinsics.h"
#include "Bot.h"
#include "DataMessage.pb.h"
#include "Network.h"
#include "SoundSourceLocalization.h"
#include "Utility.h"

#include <iostream>
#include <k4a/k4a.hpp>
#include <string>

#include <chrono>
#include <ncurses.h>
#include <thread>
#include <vector>

#include <open3d/Open3D.h>

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
    if (argc > 1) {
        config_file_path = argv[1];
    }
    etrs::utility::Config program_config(config_file_path);

    float INTERVAL_RADIAN = program_config.getFloat("interval_radian");
    float EXIT_RADIAN = program_config.getFloat("exit_radian");
    float VOXEL_SIZE = program_config.getFloat("voxel_size");
    float FINAL_VOXEL_SIZE = program_config.getFloat("final_voxel_size");
    float SMALL_RADIUS_MULTIPLIER = program_config.getFloat("small_radius_multiplier");
    float LARGE_RADIUS_MULTIPLIER = program_config.getFloat("large_radius_multiplier");
    // string WEB_SOCKET_SERVER_ADDRESS = program_config.get("web_socket_server_address");
    // int WEB_SOCKET_SERVER_PORT = program_config.getInt("web_socket_server_port");
    // string WEB_SOCKET_SERVER_PATH = program_config.get("web_socket_server_path");
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

    k4a::device device;
    // k4a_device_t device;

    etrs::bot::BotArm bot_arm(BOT_ARM_SERIAL_PORT_NAME);
    etrs::bot::BotMotor bot_motor(STM32_SERIAL_PORT_NAME);
    etrs::bot::BotCar bot_car(BOT_CAR_SERIAL_PORT_NAME, (char)0x12, 0.62);
    etrs::bot::BotLed bot_led(STM32_SERIAL_PORT_NAME);

    // 发现已连接的设备数
    if (etrs::kinect::checkKinectNum(1) == false) {
        return 0;
    }

    // bot_motor.rotate(FIRST_MOTOR_ROTATION, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });

    // 打开（默认）设备
    // device = k4a::device::open(K4A_DEVICE_DEFAULT);

    // k4a_device_open(0, &device);
    cout << "打开 Azure Kinect 设备" << endl;
    k4a_device_configuration_t config;

    // 配置并启动设备
    config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; // TODO: 试试 BGRA32
    config.color_resolution = K4A_COLOR_RESOLUTION_1536P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.synchronized_images_only = true; // 只输出同步的图像，即同步深度图和彩色图

    // k4a_device_start_cameras(device, &config);
    // device.start_cameras(&config);
    // cout << "开启相机。" << endl;

    // // 稳定化
    // etrs::kinect::stabilizeCamera(device);
    // cout << "------------------------------------" << endl;
    // cout << "----- 成功启动 Azure Kinect DK -----" << endl;
    // cout << "------------------------------------" << endl;

    // k4a::calibration k4a_calibration = device.get_calibration(config.depth_mode, config.color_resolution);

    // _k4a_calibration_t k4a_calibration;
    // k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &k4a_calibration);

    // k4a::transformation k4a_transformation = k4a::transformation(k4a_calibration);
    // k4a_transformation_t k4a_transformation = k4a_transformation_create(&k4a_calibration);

    // LED亮红
    bot_led.setLedColor(etrs::bot::BotLed::LedColor::RED);

    if (IS_CONNECT_ARM) {
        bot_arm.reset();
        Debug::CoutSuccess("机械臂复位成功");
    }

    // 创建服务器等待连接
    etrs::net::Client client(SERVER_PORT, [&]() {
        // LED亮绿
        bot_led.setLedColor(etrs::bot::BotLed::LedColor::GREEN);
    });

    // 定义互斥锁
    mutex client_mutex;
    mutex stm32_mutex;
    mutex recon_mutex;

    // FIXME:
    bool need_reconnstrcution = true;
    int flag_recording = 0;
    bool kinect_going = true;

    // etrs::proto::KinectMode::Mode kinect_mode = etrs::proto::KinectMode::REAL_TIME;
    etrs::proto::KinectMode::Mode kinect_mode = etrs::proto::KinectMode::RECONSTRCUTION;
    int angle = 90;

    // 声源定位线程
    // thread ssl_thread([&]() {
    //     if (ENABLE_SOUND_SOURCE_LOCALIZATION) {
    //         etrs::ssl::SoundSourceDetector sound_source_detector(SAMPLE_RATE, SAMPLES, CHANNELS, MICROPHONE_NAME);
    //         sound_source_detector.start();
    //         while (true) {
    //             // lock_guard<mutex> lock(ssl_mutex);
    //             Eigen::Vector3f sound_source = sound_source_detector.locate();
    //             etrs::proto::DataMessage data_message;
    //             data_message.set_type(etrs::proto::DataMessage::SOUND_SOURCE);
    //             etrs::proto::SoundSource *sound_source_message = data_message.mutable_sound_source();
    //             sound_source_message->set_x(sound_source[0]);
    //             sound_source_message->set_y(sound_source[1]);
    //             sound_source_message->set_z(sound_source[2]);
    //             Debug::CoutInfo("声源位置: {}, {}, {}", sound_source[0], sound_source[1], sound_source[2]);
    //             client.sendMessage(data_message);
    //         }
    //     }
    // });

    // 机械臂线程
    // thread bot_arm_thread([&]() {
    // unsigned char bot_arm_buffer[128];
    // while (true) {
    // lock_guard<mutex> lock(arm_mutex);
    // int len = bot_arm.recvData(bot_arm_buffer, 128);
    // 打印数据
    // for (int i = 0; i < len; i++) {
    //     cout << (int)bot_arm_buffer[i] << " ";
    // }
    // cout << endl;
    // }
    // });

    int flag_seq;

    thread receive_client_thread([&]() {
        char client_buffer[1024];
        while (true) {
            etrs::proto::DataMessage data_message;
            if (!client.recvMessage(data_message)) {
                continue;
            }
            switch ((int)data_message.type()) {
                case (int)etrs::proto::DataMessage::BOT_MOTOR: {
                    Debug::CoutSuccess("收到重建请求");
                    angle = data_message.bot_motor().angle();
                    // if (angle == 90) {
                    unique_lock<mutex> lock(recon_mutex);
                    flag_recording = 0;
                    need_reconnstrcution = true;
                    // } else if (angle == 360) {
                    //     bot_motor.rotate(FIRST_MOTOR_ROTATION, 360, MOTOR_SPEED,
                    //                      [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });
                    // }
                    break;
                }
                case (int)etrs::proto::DataMessage::BOT_CAR: {
                    Debug::CoutSuccess("收到机器人数据");
                    int seq_length = data_message.bot_car().move_sequence_size();
                    int sequence_flag = data_message.bot_car().move_sequence(0);
                    Debug::CoutDebug("序列长度: {}", seq_length);
                    float seq[seq_length];
                    for (int i = 0; i < seq_length; i++) {
                        // cout << data_message.bot_car().move_sequence(i) << " ";
                        Debug::CoutDebug("序列: {}", data_message.bot_car().move_sequence(i));
                        seq[i] = data_message.bot_car().move_sequence(i);
                    }
                    bot_car.executeMoveSequence(seq, seq_length);
                    break;
                }
                case (int)etrs::proto::DataMessage::BOT_ARM: {
                    Debug::CoutSuccess("收到机械臂数据");
                    int length = data_message.bot_arm().data_buffer().length();
                    bot_arm.execute(data_message.bot_arm().data_buffer().data(), length);
                    bot_arm.sendCommand(etrs::bot::BotArm::CommandSet::READ_ANGLE);
                    // for (int i = 0; i < length; i++) {
                    //     cout << "jxb:" << data_message.bot_arm().data_buffer().data() << endl;
                    // }
                    break;
                }
                case (int)etrs::proto::DataMessage::BOT_GRIPPER: {
                    int status = data_message.bot_gripper().status();
                    Debug::CoutSuccess("收到机械臂夹爪数据: {}", status);
                    if (status == 1) {
                        bot_arm.openGripper(0x32);
                    } else if (status == 0) {
                        bot_arm.closeGripper(0x32);
                    }
                    break;
                }
                case (int)etrs::proto::DataMessage::KINECT_MODE: {
                    Debug::CoutSuccess("收到Kinect模式切换请求");
                    // kinect_going = false;
                    // kinect_mode = data_message.kinect_mode().mode();
                    break;
                }
                case (int)etrs::proto::DataMessage::OTHER:
                default:
                    Debug::CoutError("未知的客户端数据类型");
                    break;
            }
        }
    });

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
                    // "M$F DONE", "M$R DONE"
                    // 舵机旋转反馈
                    break;
                }
                case 'T': {
                    float humi = stm32_buffer[1] + stm32_buffer[2] / 10.0;
                    float temp = stm32_buffer[3] + stm32_buffer[4] / 10.0;
                    // Debug::CoutDebug("温湿度传感器数据: {} {}", humi, temp);
                    etrs::proto::DataMessage data_message;
                    data_message.set_type(etrs::proto::DataMessage::TEMP_AND_HUMI);
                    etrs::proto::TempAndHumi *temp_and_humi = data_message.mutable_temp_and_humi();
                    temp_and_humi->set_humi(humi);
                    temp_and_humi->set_temp(temp);
                    // unique_lock<mutex> lock(client_mutex);
                    client.sendMessage(data_message);
                    break;
                }
            }
        }
    });

    // 开启深度和彩色图像的对齐
    bool enable_align_depth_to_color = true;

    // 体素网格参数
    float voxel_size = BLOCK_VOXEL_SIZE;
    float trunc_voxel_multiplier = TRUNC_VOXEL_MULTIPLIER;
    int block_resolution = BLOCK_RESOLUTION;
    int block_count = BLOCK_COUNT;

    // 里程计参数
    float depth_scale = DEPTH_SCALE;
    float depth_max = DEPTH_MAX;
    float depth_diff = DEPTH_DIFF;

    // 相机内参
    // camera::PinholeCameraIntrinsic intrinsic =
    //     camera::PinholeCameraIntrinsic(camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);

    // 设备类型
    core::Device cuda_ = core::Device("cuda:0");

    io::AzureKinectSensorConfig sensor_config;
    string azure_kinect_config_file = "../azure_kinect_sensor_conf.json";
    io::ReadIJsonConvertibleFromJSON(azure_kinect_config_file, sensor_config);
    io::AzureKinectSensor sensor(sensor_config);

    unsigned char buffer[10] = {0};
    buffer[0] = 0xff;
    buffer[1] = 0xfe;
    buffer[2] = 0x00; // A电机速度
    buffer[3] = 0x00; // B电机速度
    buffer[4] = 0x00; // A电机方向
    buffer[5] = 0x00; // B电机方向
    buffer[6] = 0x00;
    buffer[7] = 0x00;
    buffer[8] = 0x00;
    buffer[9] = 0x00;

    // initscr();
    // cbreak();
    // noecho();
    // keypad(stdscr, TRUE);
    char ch;

    thread control_thread([&]() {
        while (true) {
            cin >> ch;
            // ch = getch(); // 获取按下的键值
            if (ch == 'w' || ch == 'W') {
                std::cout << "向前移动" << std::endl;
                bot_car.moveForwardDistance(10);
            } else if (ch == 'a' || ch == 'A') {
                std::cout << "向左转动" << std::endl;
                bot_car.autoTurnByAngle(90);
            } else if (ch == 's' || ch == 'S') {
                std::cout << "向后移动" << std::endl;
                bot_car.moveBackwardDistance(10);
            } else if (ch == 'd' || ch == 'D') {
                std::cout << "向右转动" << std::endl;
                bot_car.autoTurnByAngle(-90);
            } else if (ch == 'x' || ch == 'X' || ch == ' ') {
                std::cout << "停止移动" << std::endl;
                bot_car.stopCar();
            } else if (ch == 'r' || ch == 'R') {
                bot_motor.rotate(-45, 3000);
                std::cout << "右转" << std::endl;
            } else if (ch == 'l' || ch == 'L') {
                bot_motor.rotate(45, 3000);
                std::cout << "左转" << std::endl;
            } else {
                std::cout << "未知操作" << std::endl;
            }
        }
    });

    while (true) {
        kinect_going = true;
        switch (kinect_mode) {
            case etrs::proto::KinectMode::RECONSTRCUTION: {
                bot_led.setLedColor(etrs::bot::BotLed::LedColor::GREEN);
                Debug::CoutInfo("切换至重建模式");

                this_thread::sleep_for(chrono::milliseconds(1000));

                core::Tensor intrinsic_t =
                    core::Tensor::Init<double>({{963.205, 0, 1012.87}, {0, 962.543, 777.369}, {0, 0, 1}});

                while (kinect_going) {

                    if (!need_reconnstrcution) {
                        continue;
                    }

                    need_reconnstrcution = false;

                    cout << "212" <<endl;

                    bot_motor.rotate(-angle / 2, 3000, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });

                    while (flag_recording < 1)
                        ;

                    bot_motor.rotate(angle, 10000, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });

                    while (flag_recording < 2)
                        ;

                    io::ReadTriangleMeshOptions options;
                    options.enable_post_processing = true;
                    options.print_progress = true;

                    geometry::TriangleMesh mesh;
                    io::ReadTriangleMesh("ply/recon.ply", mesh, options);
                    Debug::CoutDebug("读取完成");

                    this_thread::sleep_for(chrono::milliseconds(1000));

                    bot_motor.rotate(-angle / 2, 3000, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });

                    Debug::CoutDebug("开始发送数据");
                    client.sendMessageFromMesh(mesh, 800);

                    while (flag_recording < 3)
                        ;
                }
                break;
            }
            case etrs::proto::KinectMode::REAL_TIME: {
                Debug::CoutInfo("切换至实时模式");
                break;
            }
            default: {
                Debug::CoutError("未知的 Kinect 模式");
                break;
            }
        }
    }

    return 1;
}
