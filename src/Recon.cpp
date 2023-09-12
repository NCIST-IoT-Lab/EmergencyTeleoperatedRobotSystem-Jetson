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

int main(int argc, char **argv) { // TODO: 可以传参，传入配置文件路径
    // 读取配置文件
    string config_file_path = "../default.conf";
    // string ans;
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

    k4a::device device;

    etrs::bot::BotArm bot_arm(BOT_ARM_SERIAL_PORT_NAME);
    etrs::bot::BotMotor bot_motor(STM32_SERIAL_PORT_NAME);
    etrs::bot::BotCar bot_car(BOT_CAR_SERIAL_PORT_NAME, (char)0x12, 0.62);
    etrs::bot::BotLed bot_led(STM32_SERIAL_PORT_NAME);

    // 发现已连接的设备数
    if (etrs::kinect::checkKinectNum(1) == false) {
        return 0;
    }

    // 定义互斥锁
    mutex client_mutex;
    mutex stm32_mutex;
    mutex recon_mutex;

    // FIXME:
    bool need_reconnstrcution = true;
    int flag_recording = 0;
    bool kinect_going = true;

    etrs::proto::KinectMode::Mode kinect_mode = etrs::proto::KinectMode::RECONSTRCUTION;

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

    // 体素网格参数
    float voxel_size = BLOCK_VOXEL_SIZE;
    float trunc_voxel_multiplier = TRUNC_VOXEL_MULTIPLIER;
    int block_resolution = BLOCK_RESOLUTION;
    int block_count = BLOCK_COUNT;

    // 里程计参数
    float depth_scale = DEPTH_SCALE;
    float depth_max = DEPTH_MAX;
    float depth_diff = DEPTH_DIFF;

    // 设备类型
    core::Device cuda_ = core::Device("cuda:0");

    io::AzureKinectSensorConfig sensor_config;
    string azure_kinect_config_file = "../azure_kinect_sensor_conf.json";
    io::ReadIJsonConvertibleFromJSON(azure_kinect_config_file, sensor_config);
    io::AzureKinectSensor sensor(sensor_config);

    while (true) {
        kinect_going = true;

        bot_motor.rotate(-an, 3000, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });

        core::Tensor intrinsic_t =
            core::Tensor::Init<double>({{963.205, 0, 1012.87}, {0, 962.543, 777.369}, {0, 0, 1}});

        sensor.Connect(0);
        Debug::CoutSuccess("相机初始化成功");

        std::shared_ptr<geometry::RGBDImage> im_rgbd;

        // 读取第一个有效帧
        do {
            im_rgbd = sensor.CaptureFrame(true);
        } while (im_rgbd == nullptr);

        // 初始化 SLAM 模型
        core::Tensor T_frame_to_model = core::Tensor::Eye(4, core::Dtype::Float64, core::Device("CPU:0"));
        t::pipelines::slam::Model model(voxel_size, block_resolution, block_count, T_frame_to_model, cuda_);

        // 读取深度图像帧
        // 使用t::geometry::Image的静态方法FromLegacy，将geometry::Image转换为t::geometry::Image
        // t::geometry::Image ref_depth = t::geometry::Image::FromLegacy(im_rgbd->depth_, cuda_);
        t::pipelines::slam::Frame input_frame(im_rgbd->depth_.height_, im_rgbd->depth_.width_, intrinsic_t, cuda_);
        t::pipelines::slam::Frame raycast_frame(im_rgbd->depth_.height_, im_rgbd->depth_.width_, intrinsic_t, cuda_);

        int i = 0;

        bot_motor.rotate(an * 2, MOTOR_SPEED, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });

        while (flag_recording == 1) {
            cout << flag_recording << endl;

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
                    result = model.TrackFrameToModel(input_frame, raycast_frame, depth_scale, depth_max, depth_diff);

                    core::Tensor t1 = etrs::utility::Transformation::RemoveYTranslationT(result.transformation_);

                    core::Tensor t2 = etrs::utility::Transformation::RemoveXZRotationT(t1, "F");
                    double translation_norm = etrs::utility::Transformation::CalculateTranslationNormT(t2);

                    if (translation_norm < 0.15) {
                        T_frame_to_model = T_frame_to_model.Matmul(t2);
                    } else {
                        tracking_success = false;
                        Debug::CoutError("里程计跟踪失败！");
                    }
                } catch (const runtime_error &e) {
                    Debug::CoutError("{}", e.what());
                    tracking_success = false;
                }
            }

            if (tracking_success) {
                model.UpdateFramePose(i, T_frame_to_model);
                model.Integrate(input_frame, depth_scale, depth_max, trunc_voxel_multiplier);
                model.SynthesizeModelFrame(raycast_frame, depth_scale, 0.1, depth_max, trunc_voxel_multiplier, false);
                i++;
            }
        }
        sensor.Disconnect();

        auto mesh = model.ExtractTriangleMesh().ToLegacy();

        Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
        Eigen::Matrix3d r = Eigen::AngleAxisd(-an / 180.0 * M_PI, Eigen::Vector3d(0, 1, 0)).toRotationMatrix();
        mesh.Rotate(r, center);

        // 发送面片数据
        Debug::CoutDebug("保存面片数据中");
        io::WriteTriangleMesh("ply/recon.ply", mesh);

        this_thread::sleep_for(chrono::milliseconds(1000));

        bot_motor.rotate(-an, 3000, [&]() { onRotated(program_config, FIRST_MOTOR_ROTATION); });

        while (flag_recording < 3)
            ;
        break;
    }
    return 1;
}
