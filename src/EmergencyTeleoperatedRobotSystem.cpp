//
// Created by Cassius0924 on 2020/03/03.
//

/*
 * SingleAzureKinect3DReconstruction
 * 此项目基于 Open3D 和 Azure Kinect DK 实现了三维重建。利用 Azure Kinect DK
 * 捕获图像并记录 IMU 数据，利用 Open3D 实现三维重建。
 */

#include "AzureKinect.h"
#include "AzureKinectExtrinsics.h"
#include "Bluetooth.h"
#include "Bot.h"
#include "DataMessage.pb.h"
#include "Network.h"
#include "SoundSourceLocalization.h"
#include "Utility.h"

#include <iostream>
#include <k4a/k4a.hpp>
#include <string>

#include <chrono>
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

int main(int argc, char **argv) {
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
    string LEFT_BOT_ARM_MAC_ADDRESS = program_config.get("left_bot_arm_mac_address");
    string RIGHT_BOT_ARM_MAC_ADDRESS = program_config.get("right_bot_arm_mac_address");
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

    // etrs::bot::BotArm bot_arm_left(BOT_ARM_SERIAL_PORT_NAME, "机械臂");
    // TODO: 将Unity端的机械臂数据的识别帧部分删除，在此处添加识别帧
    etrs::bot::BotArm bot_arm_left(LEFT_BOT_ARM_MAC_ADDRESS, "左机械臂");
    etrs::bot::BotArm bot_arm_right(RIGHT_BOT_ARM_MAC_ADDRESS, "右机械臂");

    etrs::bot::BotMotor bot_motor(STM32_SERIAL_PORT_NAME);
    etrs::bot::BotCar bot_car(BOT_CAR_SERIAL_PORT_NAME, (char)0x12, 0.62);
    etrs::bot::BotLed bot_led(STM32_SERIAL_PORT_NAME);

    // 发现已连接的设备数
    if (IS_CONNECT_KINECT && etrs::kinect::checkKinectNum(1) == false) {
        return 0;
    }

    // bot_motor.rotate(FIRST_MOTOR_ROTATION, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });

    // 打开（默认）设备
    // device = k4a::device::open(K4A_DEVICE_DEFAULT);

    // k4a_device_open(0, &device);
    // cout << "打开 Azure Kinect 设备" << endl;
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

    if (IS_CONNECT_ARM) {
        bot_arm_left.reset();
        bot_arm_right.reset();
        Debug::CoutSuccess("机械臂复位成功");
    }

    // thread receive_bot_arm_thread([&]() {
    //     char bot_arm_buffer[64];
    //     while (true) {
    //         int len = -1;
    //         if ((len = bot_arm_left.recvData(bot_arm_buffer, 64)) < 0) {
    //             cout << 1 << endl;
    //             continue;
    //         };
    //         cout << "bot_arm_left: ";
    //         // 打印
    //         for (int i = 0; i < len; i++) {
    //             cout << (int)bot_arm_buffer[i] << " ";
    //         }
    //         cout << endl;
    //     }
    // });

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

    // LED亮红
    bot_led.setLedColor(etrs::bot::BotLed::LedColor::RED);

    // 创建服务器等待连接
    etrs::net::HoloCommunicator client(SERVER_PORT);
    client.createServerSocket();
    client.acceptConnection([&]() {
        // LED亮绿
        bot_led.setLedColor(etrs::bot::BotLed::LedColor::GREEN);
        Debug::CoutSuccess("HoloCommunicator 连接成功");
    });

    etrs::net::HoloCommunicator client1(SERVER_PORT, client.server_socket_fd);
    // client1.createServerSocket();
    // client1.acceptConnection([&]() {
    //     Debug::CoutSuccess("HoloCommunicator 2 连接成功");
    // });

    // 定义互斥锁
    mutex client_mutex;
    mutex stm32_mutex;
    mutex recon_mutex;

    // FIXME:
    bool need_reconnstrcution = true;
    int flag_recording = 0;
    bool kinect_going = true;

    // TODO: 实时模式，留给下一届吧
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
                    angle = data_message.bot_motor().angle(); // TODO: 保证angle为2的倍数
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
                    Debug::CoutSuccess("收到底盘车数据");
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
                    int length = data_message.bot_arm().angles_size();
                    switch ((int)data_message.bot_arm().side()) {
                        case (int)etrs::proto::BotArm::Left: {
                            // Debug::CoutDebug("左机械臂");
                            bot_arm_left.executeByAngle(data_message.bot_arm().angles().data());
                            // bot_arm_left.sendCommand(etrs::bot::BotArm::CommandTypeSet::READ_ANGLE);
                            // int length = data_message.bot_arm().data_buffer().length();
                            break;
                        }
                        case (int)etrs::proto::BotArm::Right: {
                            // Debug::CoutDebug("右机械臂");
                            bot_arm_right.executeByAngle(data_message.bot_arm().angles().data());
                            break;
                        }
                        default: {
                            Debug::CoutError("未知侧的机械臂");
                        }
                    }
                    break;
                }
                case (int)etrs::proto::DataMessage::BOT_GRIPPER: {
                    Debug::CoutSuccess("收到机械臂夹爪数据");
                    switch ((int)data_message.bot_gripper().side()) {
                        case (int)etrs::proto::BotGripper::Left: {
                            int status = data_message.bot_gripper().status();
                            if (status == 1) {
                                bot_arm_left.openGripper(0x32);
                            } else if (status == 0) {
                                bot_arm_left.closeGripper(0x32);
                            }
                            break;
                        }
                        case (int)etrs::proto::BotGripper::Right: {
                            int status = data_message.bot_gripper().status();
                            if (status == 1) {
                                // bot_arm_right.openGripper(0x32);
                            } else if (status == 0) {
                                // bot_arm_right.closeGripper(0x32);
                            }
                            break;
                        }
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

    // TODO: 将接收线程写进类里，这样可以在写阻塞式的motor.rotateWait()
    thread receive_stm32_thread([&]() {
        unsigned char stm32_buffer[32];
        while (true) {
            unique_lock<mutex> lock(stm32_mutex);
            // 等待接受到stm32的数据
            bot_motor.recvData(stm32_buffer, 32);
            char data_type = stm32_buffer[0];
            // Debug::CoutDebug("OKOKOK，STM32: {}", data_type);
            // 解析stm32数据
            switch (data_type) {
                case 'M':
                case 'm': {
                    if ((stm32_buffer[4] == 'd' || stm32_buffer[4] == 'D') &&
                        (stm32_buffer[5] == 'o' || stm32_buffer[5] == 'O') &&
                        (stm32_buffer[6] == 'n' || stm32_buffer[6] == 'N') &&
                        (stm32_buffer[7] == 'e' || stm32_buffer[7] == 'E')) {
                        Debug::CoutDebug("舵机旋转完成");
                        ++flag_recording;
                    }
                    // "M$F DONE", "M$R DONE"
                    // 舵机旋转反馈
                    break;
                }
                case 'T':
                case 't': {
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

    geometry::TriangleMesh llm;
    // io::ReadTriangleMesh("ply/sm.ply", llm);
    Debug::CoutDebug("开始发送数据");
    client.sendMessageFromMesh(llm, 800);

    if (!IS_CONNECT_KINECT) {
        while (true)
            ;
    }

    io::AzureKinectSensorConfig sensor_config;
    string azure_kinect_config_file = "../azure_kinect_sensor_conf.json";
    io::ReadIJsonConvertibleFromJSON(azure_kinect_config_file, sensor_config);
    io::AzureKinectSensor sensor(sensor_config);

    while (true) {
        kinect_going = true;
        switch (kinect_mode) {
            case etrs::proto::KinectMode::RECONSTRCUTION: {
                Debug::CoutInfo("切换至重建模式");
                core::Tensor intrinsic_t =
                    core::Tensor::Init<double>({{963.205, 0, 1012.87}, {0, 962.543, 777.369}, {0, 0, 1}});

                while (kinect_going) {
                    if (!need_reconnstrcution) {
                        continue;
                    }

                    need_reconnstrcution = false;

                    bot_motor.rotate(-angle / 2, 3000, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });
                    this_thread::sleep_for(chrono::seconds(5));
                    bot_motor.rotate(angle, 10000, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });
                    this_thread::sleep_for(chrono::seconds(15));
                    bot_motor.rotate(-angle / 2, 3000, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });

                    if (0) {
                        this_thread::sleep_for(chrono::seconds(2));
                        bot_motor.rotate(-angle / 2, 3000, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });

                        while (flag_recording < 1)
                            ;
                        // device.close();

                        sensor.Connect(0);
                        Debug::CoutSuccess("相机初始化成功");

                        std::shared_ptr<geometry::RGBDImage> im_rgbd;

                        // 读取第一个有效帧
                        do {
                            im_rgbd = sensor.CaptureFrame(true);
                        } while (im_rgbd == nullptr);

                        // 初始化 SLAM 模型
                        core::Tensor T_frame_to_model =
                            core::Tensor::Eye(4, core::Dtype::Float64, core::Device("CPU:0"));
                        t::pipelines::slam::Model model(voxel_size, block_resolution, block_count, T_frame_to_model,
                                                        cuda_);

                        t::pipelines::slam::Frame input_frame(im_rgbd->depth_.height_, im_rgbd->depth_.width_,
                                                              intrinsic_t, cuda_);
                        t::pipelines::slam::Frame raycast_frame(im_rgbd->depth_.height_, im_rgbd->depth_.width_,
                                                                intrinsic_t, cuda_);

                        int i = 0;

                        // 旋转电机
                        bot_motor.rotate(angle, MOTOR_SPEED,
                                         [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });
                        while (flag_recording < 2) {
                            im_rgbd = sensor.CaptureFrame(true);
                            if (im_rgbd == nullptr) { // 读取失败则跳过
                                continue;
                            }

                            Debug::CoutInfo("处理中: {}", i);

                            input_frame.SetDataFromImage("depth",
                                                         t::geometry::Image::FromLegacy(im_rgbd->depth_, cuda_));
                            input_frame.SetDataFromImage("color",
                                                         t::geometry::Image::FromLegacy(im_rgbd->color_, cuda_));

                            // 里程计跟踪
                            bool tracking_success = true;

                            if (i > 0) {
                                t::pipelines::odometry::OdometryResult result;
                                try {
                                    result = model.TrackFrameToModel(input_frame, raycast_frame, depth_scale, depth_max,
                                                                     depth_diff);
                                    // TODO: 打印 result.transformation_
                                    // 的值，看看位移值是否为0，再比较一下旋转值和IMU获取的旋转值是否一致？

                                    core::Tensor t1 =
                                        etrs::utility::Transformation::RemoveYTranslationT(result.transformation_);

                                    // string d = FIRST_MOTOR_ROTATION == "F" ? "R" : "F";
                                    core::Tensor t2 = etrs::utility::Transformation::RemoveXZRotationT(t1, "F");
                                    double translation_norm =
                                        etrs::utility::Transformation::CalculateTranslationNormT(t2);

                                    if (translation_norm < 0.15) {
                                        T_frame_to_model = T_frame_to_model.Matmul(t2);
                                    } else {
                                        tracking_success = false;
                                        Debug::CoutError("里程计跟踪失败！");
                                    }
                                    // Debug::CoutInfo("fitness: {}， translation_norm: {}", result.fitness_,
                                    // translation_norm);

                                } catch (const runtime_error &e) {
                                    Debug::CoutError("{}", e.what());
                                    tracking_success = false;
                                    --i;
                                }
                            }

                            if (tracking_success) {
                                model.UpdateFramePose(i, T_frame_to_model);
                                model.Integrate(input_frame, depth_scale, depth_max, trunc_voxel_multiplier);
                                model.SynthesizeModelFrame(raycast_frame, depth_scale, 0.1, depth_max,
                                                           trunc_voxel_multiplier, false);
                                i++;
                            }
                        }
                        sensor.Disconnect();

                        // TODO: tensor旋转是否可用?
                        core::Tensor rotate_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(
                            Eigen::AngleAxisd(-(angle / 2) / 180.0 * M_PI, Eigen::Vector3d(0, 1, 0))
                                .toRotationMatrix());
                        core::Tensor center_tensor =
                            core::Tensor::Zeros({3}, core::Dtype::Float64, core::Device("CPU:0"));
                        auto mesh = model.ExtractTriangleMesh().Rotate(rotate_tensor, center_tensor);
                        // 点云数据
                        // auto point_cloud = model.ExtractPointCloud();
                        auto legacy_mesh = mesh.ToLegacy();
                    }

                    // FIXME:
                    // 发送面片数据
                    // Debug::CoutDebug("保存面片数据中");
                    // io::WriteTriangleMesh("ply/sm.ply", legacy_mesh);

                    geometry::TriangleMesh legacy_mesh1;
                    io::ReadTriangleMesh("ply/sm.ply", legacy_mesh1);

                    // bot_motor.rotate(-angle / 2, 3000, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });
                    // while (flag_recording < 3)
                    //     ;

                    Debug::CoutDebug("开始发送数据");
                    client.sendMessageFromMesh(legacy_mesh1, 800);
                    // client1.sendMessageFromMesh(legacy_mesh1, 800);
                }
                break;
            }
            case etrs::proto::KinectMode::REAL_TIME: {
                Debug::CoutInfo("切换至实时模式");
                while (kinect_going) {
                    // FIXME:
                    // sensor.Disconnect();

                    open3d::geometry::PointCloud cloud;

                    k4a::capture capture;
                    device.get_capture(&capture);

                    k4a::image rgb_image_item = capture.get_color_image();
                    k4a::image depth_image_item = capture.get_depth_image();

                    int color_image_width_pixels = rgb_image_item.get_width_pixels();
                    int color_image_height_pixels = rgb_image_item.get_height_pixels();

                    k4a::image transformed_depthImage =
                        k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, color_image_width_pixels,
                                           color_image_height_pixels, color_image_width_pixels * (int)sizeof(uint16_t));
                    k4a::image point_cloud_image =
                        k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM, color_image_width_pixels, color_image_height_pixels,
                                           color_image_width_pixels * 3 * (int)sizeof(int16_t));

                    // k4a_transformation.depth_image_to_color_camera(depth_image_item, &transformed_depthImage);

                    // k4a_transformation.depth_image_to_point_cloud(transformed_depthImage,
                    // K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

                    cloud.points_.resize(color_image_width_pixels * color_image_height_pixels);
                    cloud.colors_.resize(color_image_width_pixels * color_image_height_pixels);

                    const int16_t *point_cloud_image_data =
                        reinterpret_cast<const int16_t *>(point_cloud_image.get_buffer());
                    const uint8_t *color_image_data = rgb_image_item.get_buffer();

                    for (int i = 0; i < color_image_width_pixels * color_image_height_pixels; i++) {
                        if (point_cloud_image_data[3 * i + 0] != 0 && point_cloud_image_data[3 * i + 1] != 0 &&
                            point_cloud_image_data[3 * i + 2] != 0) {
                            cloud.points_[i] = Eigen::Vector3d(point_cloud_image_data[3 * i + 0] / 1000.0f,
                                                               point_cloud_image_data[3 * i + 1] / 1000.0f,
                                                               point_cloud_image_data[3 * i + 2] / 1000.0f);
                            cloud.colors_[i] = Eigen::Vector3d(color_image_data[4 * i + 2] / 255.0f,
                                                               color_image_data[4 * i + 1] / 255.0f,
                                                               color_image_data[4 * i + 0] / 255.0f);
                        } else {
                            cloud.points_[i] = Eigen::Vector3d::Zero();
                            cloud.colors_[i] = Eigen::Vector3d::Zero();
                        }
                    }

                    // k4a::capture capture;

                    // Debug::CoutDebug("实时 1 帧");
                    // 将iamge转点云
                    // auto cloud = *geometry::PointCloud::CreateFromRGBDImage(
                    //     *image, camera::PinholeCameraIntrinsic(
                    //                 camera::PinholeCameraIntrinsicParameters::Kinect2DepthCameraDefault));

                    auto point_cloud = *cloud.VoxelDownSample(0.03);

                    geometry::KDTreeSearchParamHybrid kd_tree_param(0.03 * 2, 30);

                    point_cloud.EstimateNormals(kd_tree_param);

                    // point_cloud转mesh
                    vector<double> distances = point_cloud.ComputeNearestNeighborDistance();
                    // 计算平均距离
                    double avg_dist = accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
                    // 设置搜索半径
                    double radius = avg_dist * LARGE_RADIUS_MULTIPLIER;
                    vector<double> radii = {radius, radius * 2};
                    auto mesh = geometry::TriangleMesh::CreateFromPointCloudBallPivoting(point_cloud, radii);

                    Debug::CoutDebug("开始发送数据");
                    client.sendMessageFromMesh(mesh, 800);
                }
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
