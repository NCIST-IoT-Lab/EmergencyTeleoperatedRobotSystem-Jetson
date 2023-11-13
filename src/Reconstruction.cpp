//
// Created by HoChihchou on 2020/03/03.
//

#include "AzureKinect.h"
#include "AzureKinectExtrinsics.h"
#include "Bluetooth.h"
#include "Bot.h"
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

    etrs::bot::BotMotor bot_motor(STM32_SERIAL_PORT_NAME);
    etrs::bot::BotCar bot_car(BOT_CAR_SERIAL_PORT_NAME, (char)0x12, 0.62);

    // 发现已连接的设备数
    if (etrs::kinect::checkKinectNum(1) == false) {
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

    int angle = 360;

    bool scan_flag = false;
    char ch;
    thread control_thread([&]() {
        while (true) {
            cin >> ch;
            if (ch == 'w' || ch == 'W') {
                Debug::CoutInfo("向前移动");
                bot_car.moveForwardDistance(10);
            } else if (ch == 'a' || ch == 'A') {
                Debug::CoutInfo("向左转动");
                bot_car.autoTurnByAngle(90);
            } else if (ch == 's' || ch == 'S') {
                Debug::CoutInfo("向后移动");
                bot_car.moveBackwardDistance(10);
            } else if (ch == 'd' || ch == 'D') {
                Debug::CoutInfo("向右转动");
                bot_car.autoTurnByAngle(-90);
            } else if (ch == 'x' || ch == 'X' || ch == ' ') {
                Debug::CoutInfo("停止移动");
                bot_car.stopCar();
            } else if (ch == 'r' || ch == 'R') {
                bot_motor.rotate(-45, 3000);
                Debug::CoutInfo("电机右转45度");
            } else if (ch == 'l' || ch == 'L') {
                bot_motor.rotate(45, 3000);
                Debug::CoutInfo("电机左转45度");
            } else if (ch == 'O') {
                Debug::CoutInfo("开始扫描环境");
                scan_flag = true;
            } else if (ch == 'm') {
                Debug::CoutInfo("请输入环境扫描角度：");
                cin >> angle;
                Debug::CoutSuccess("设置成功！环境扫描角度为：{}", angle);
                Debug::CoutInfo("输入“O”开始扫描环境...");
            } else {
                std::cout << "未知操作" << std::endl;
            }
        }
    });

    mutex stm32_mutex;
    mutex recon_mutex;

    // FIXME:
    int flag_recording = 0;
    bool kinect_going = true;

    thread receive_stm32_thread([&]() {
        unsigned char stm32_buffer[32];
        while (true) {
            unique_lock<mutex> lock(stm32_mutex);
            // 等待接受到stm32的数据
            bot_motor.recvData(stm32_buffer, 32);
            char data_type = stm32_buffer[0];
            // this_thread::sleep_for(chrono::microseconds(500));
            // 转string
            // string stm32_data = "";
            // for (int i = 0; i < 32; ++i) {
            //     stm32_data += stm32_buffer[i];
            // }
            // Debug::CoutDebug("stm32_data: {}", stm32_data);

            // 解析stm32数据
            switch (data_type) {
                case 'M':
                case 'm': {
                    if ((stm32_buffer[4] == 'd' || stm32_buffer[4] == 'D') &&
                        (stm32_buffer[5] == 'o' || stm32_buffer[5] == 'O') &&
                        (stm32_buffer[6] == 'n' || stm32_buffer[6] == 'N') &&
                        (stm32_buffer[7] == 'e' || stm32_buffer[7] == 'E')) {
                        Debug::CoutDebug("步进电机旋转完毕！");
                        ++flag_recording;
                    }
                    // "M$F DONE", "M$R DONE"
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

    io::AzureKinectSensorConfig sensor_config;
    string azure_kinect_config_file =
        "/home/ncistwlwsys/hezhizhou-projects/disk/SingleAzureKinect3DReconstruction/azure_kinect_sensor_conf.json";
    io::ReadIJsonConvertibleFromJSON(azure_kinect_config_file, sensor_config);
    io::AzureKinectSensor sensor(sensor_config);

    int angle_temp;
    char c;
    while (true) {
        kinect_going = true;
        core::Tensor intrinsic_t =
            core::Tensor::Init<double>({{963.205, 0, 1012.87}, {0, 962.543, 777.369}, {0, 0, 1}});

        Debug::CoutInfo("初始化系统中...");
        this_thread::sleep_for(chrono::seconds(5));

        while (kinect_going) {
            Debug::CoutInfo("输入“O”开始扫描环境...");
            while (!scan_flag)
                ;
            scan_flag = false;
            flag_recording = 0;
            // Debug::CoutInfo("请输入扫描角度（默认为360）:");
            // cin >> angle_temp;
            // if (angle_temp > 0 && angle_temp <= 360) {
            //     angle = angle_temp;
            // } else {
            //     Debug::CoutWarning("扫描角度为不正确，将使用默认值360度");
            //     angle = 360;
            // }

            bot_motor.rotate(-angle / 2, 3000, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });

            sensor.Connect(0);
            Debug::CoutSuccess("相机初始化成功");

            while (flag_recording < 1)
                ;

            std::shared_ptr<geometry::RGBDImage> im_rgbd;

            // 读取第一个有效帧
            do {
                im_rgbd = sensor.CaptureFrame(true);
            } while (im_rgbd == nullptr);

            // 初始化 SLAM 模型
            core::Tensor T_frame_to_model = core::Tensor::Eye(4, core::Dtype::Float64, core::Device("CPU:0"));
            t::pipelines::slam::Model model(voxel_size, block_resolution, block_count, T_frame_to_model, cuda_);

            t::pipelines::slam::Frame input_frame(im_rgbd->depth_.height_, im_rgbd->depth_.width_, intrinsic_t, cuda_);
            t::pipelines::slam::Frame raycast_frame(im_rgbd->depth_.height_, im_rgbd->depth_.width_, intrinsic_t,
                                                    cuda_);

            int i = 0;

            // 旋转电机
            bot_motor.rotate(angle, MOTOR_SPEED, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });

            while (flag_recording < 2) {
                im_rgbd = sensor.CaptureFrame(true);
                if (im_rgbd == nullptr) { // 读取失败则跳过
                    continue;
                }

                Debug::CoutInfo("处理中: {}", i);

                input_frame.SetDataFromImage("depth", t::geometry::Image::FromLegacy(im_rgbd->depth_, cuda_));
                input_frame.SetDataFromImage("color", t::geometry::Image::FromLegacy(im_rgbd->color_, cuda_));

                // 里程计跟踪
                bool tracking_success = true;

                if (i > 0) {
                    t::pipelines::odometry::OdometryResult result;
                    try {
                        result =
                            model.TrackFrameToModel(input_frame, raycast_frame, depth_scale, depth_max, depth_diff);
                        // TODO: 打印 result.transformation_
                        // 的值，看看位移值是否为0，再比较一下旋转值和IMU获取的旋转值是否一致？

                        core::Tensor t1 = etrs::utility::Transformation::RemoveYTranslationT(result.transformation_);

                        // string d = FIRST_MOTOR_ROTATION == "F" ? "R" : "F";
                        core::Tensor t2 = etrs::utility::Transformation::RemoveXZRotationT(t1, "F");
                        double translation_norm = etrs::utility::Transformation::CalculateTranslationNormT(t2);

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
                    model.SynthesizeModelFrame(raycast_frame, depth_scale, 0.1, depth_max, trunc_voxel_multiplier,
                                               false);
                    i++;
                }
            }
            sensor.Disconnect();

            // TODO: tensor旋转是否可用?
            core::Tensor rotate_tensor = open3d::core::eigen_converter::EigenMatrixToTensor(
                Eigen::AngleAxisd(-(angle / 2) / 180.0 * M_PI, Eigen::Vector3d(0, 1, 0)).toRotationMatrix());
            core::Tensor center_tensor = core::Tensor::Zeros({3}, core::Dtype::Float64, core::Device("CPU:0"));
            auto point_cloud = model.ExtractPointCloud().Rotate(rotate_tensor, center_tensor);

            bot_motor.rotate(-angle / 2, 3000, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });

            // 保存数据
            // 获取当前时间
            Debug::CoutDebug("保存点云文件中...");
            string time_string = Timer::getCurrentTime();
            string file_name = CLOUD_FILE_PATH + time_string + ".ply";
            open3d::t::io::WritePointCloud(file_name, point_cloud);
            Debug::CoutSuccess("点云文件 {} 保存成功！", file_name);
        }
        break;
    }

    return 1;
}
