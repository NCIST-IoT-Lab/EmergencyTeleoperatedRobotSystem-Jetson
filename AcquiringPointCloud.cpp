/*
 * SingleAzureKinect3DReconstruction
 * 此项目基于 Open3D 和 Azure Kinect DK 实现了三维重建。利用 Azure Kinect DK 捕获图像并记录 IMU 数据，利用 Open3D 实现三维重建。
 *
 * Created by Cassius0924 on 2020/03/03.
 *
 */

#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include <thread>

#include <k4a/k4a.hpp>

// Open3D
#include <open3d/Open3D.h>

// WebSocket
//#include "CasWebSocket.h"

#include "CasAzureKinectExtrinsics.h"

using namespace std;

int main(int argc, char *argv[]) {
    // 发现已连接的设备数
    const uint32_t device_count = k4a::device::get_installed_count();
    if (0 == device_count) {
        std::cout << "Error: no K4A devices found. " << std::endl;
        return -1;
    } else {
        std::cout << "Found " << device_count << " connected devices. " << std::endl;
        if (1 != device_count)// 超过1个设备，也输出错误信息。
        {
            std::cout << "Error: more than one K4A devices found. " << std::endl;
            return -1;
        } else// 该示例代码仅限对1个设备操作
        {
            std::cout << "Done: found 1 K4A device. " << std::endl;
        }

    }
    // 打开（默认）设备
    k4a::device device = k4a::device::open(K4A_DEVICE_DEFAULT);
    std::cout << "Done: open device. " << std::endl;

    // 配置并启动设备
    k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    //config.camera_fps = K4A_FRAMES_PER_SECOND_15;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    //config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    config.synchronized_images_only = true;// ensures that depth and color images are both available in the capture
    device.start_cameras(&config);
    std::cout << "Done: start camera." << std::endl;

    // 稳定化
    k4a::capture capture;
    int i_auto = 0;//用来稳定，类似自动曝光
    int i_auto_error = 0;// 统计自动曝光的失败次数
    while (true) {
        if (device.get_capture(&capture)) {
            std::cout << i_auto << ". Capture several frames to give auto-exposure" << std::endl;
            // 跳过前 n 个（成功的数据采集）循环，用来稳定
            if (i_auto != 30) {
                i_auto++;
                continue;
            } else {
                std::cout << "Done: auto-exposure" << std::endl;
                break;// 跳出该循环，完成相机的稳定过程
            }

        } else {
            std::cout << i_auto_error << ". K4A_WAIT_RESULT_TIMEOUT." << std::endl;
            if (i_auto_error != 30) {
                i_auto_error++;
                continue;
            } else {
                std::cout << "Error: failed to give auto-exposure. " << std::endl;
                return -1;
            }
        }
    }
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "----- Have Started Kinect DK. -----" << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    // 从设备获取捕获
    k4a::image rgb_image;
    k4a::image depth_image;
    k4a::image transformed_depthImage;
    k4a::image point_cloud_image;

    //先创建个WebSocket对象
    //服务器地址：ws://175.178.56.40:8080/ws-test
//    WebSocket webSocket("175.178.56.40","8080","/ws-test");
//    // WebSocket webSocket("ws://39.108.216.190/ws");
//    //连接服务器
//    if (webSocket.connect()) {
//        cout << "connect success" << endl;
//    } else {
//        cout << "connect failed" << endl;
//    }

    //启动 IMU
    k4a_imu_sample_t imu_sample;
    device.start_imu();

//    float prev_roll = 100;
//    float prev_pitch = 100;
    float prev_yaw = 100;

    // 定义欧拉角
    cas::EulerAngle prev_angle(0, 0, 0);
    // 定义最终的点云
    auto final_cloud = std::make_shared<open3d::geometry::PointCloud>();
    float temp = 0;
    int flag = 0;

    // 双线程运行，一个线程用来获取IMU数据，一个线程用来计算点云数据

    // 定义互斥锁
    std::mutex mutex;

    // 定义条件变量
    std::condition_variable cv;

    // 定义共享变量
//    bool need_cal_cloud = false;
    bool ready_to_break = false;
    bool need_break = false;

    int task_count = 0;

    // 定义相机线程
    std::thread camera_thread([&]() {
        while (true) {
            // 获取互斥锁
            std::unique_lock<std::mutex> lock(mutex);
            // 等待条件变量
            while (task_count <= 0) {
                cv.wait(lock);
            }
            // 释放互斥锁
            lock.unlock();

            // 获取当前时刻的欧拉角
            cas::EulerAngle cur_angle = prev_angle;

            device.get_capture(&capture);
            rgb_image = capture.get_color_image();
            depth_image = capture.get_depth_image();

            k4a::calibration k4a_calibration = device.get_calibration(config.depth_mode, config.color_resolution);
            k4a::transformation k4a_transformation = k4a::transformation(k4a_calibration);

            int color_image_width_pixels = rgb_image.get_width_pixels();
            int color_image_height_pixels = rgb_image.get_height_pixels();
            transformed_depthImage = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
                                                        color_image_width_pixels,
                                                        color_image_height_pixels,
                                                        color_image_width_pixels * (int) sizeof(uint16_t));
            point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
                                                   color_image_width_pixels,
                                                   color_image_height_pixels,
                                                   color_image_width_pixels * 3 * (int) sizeof(int16_t));

            if (depth_image.get_width_pixels() == rgb_image.get_width_pixels() &&
                depth_image.get_height_pixels() == rgb_image.get_height_pixels()) {
                std::copy(depth_image.get_buffer(),
                          depth_image.get_buffer() +
                          depth_image.get_height_pixels() * depth_image.get_width_pixels() *
                          (int) sizeof(uint16_t), transformed_depthImage.get_buffer());
                cout << "if" << endl;
            } else {
                cout << "else" << endl;
                k4a_transformation.depth_image_to_color_camera(depth_image, &transformed_depthImage);
            }
            k4a_transformation.depth_image_to_point_cloud(transformed_depthImage, K4A_CALIBRATION_TYPE_COLOR,
                                                          &point_cloud_image);

            auto cloud = make_shared<open3d::geometry::PointCloud>();

            const int16_t *point_cloud_image_data = reinterpret_cast<const int16_t *>(point_cloud_image.get_buffer());
            const uint8_t *color_image_data = rgb_image.get_buffer();

            for (int i = 0; i < color_image_width_pixels * color_image_height_pixels; i++) {
                if (point_cloud_image_data[3 * i + 0] != 0 && point_cloud_image_data[3 * i + 1] != 0 &&
                    point_cloud_image_data[3 * i + 2] != 0) {
                    cloud->points_.push_back(Eigen::Vector3d(point_cloud_image_data[3 * i + 0] / 1000.0f,
                                                             point_cloud_image_data[3 * i + 1] / 1000.0f,
                                                             point_cloud_image_data[3 * i + 2] / 1000.0f));
                    cloud->colors_.push_back(Eigen::Vector3d(color_image_data[4 * i + 2] / 255.0f,
                                                             color_image_data[4 * i + 1] / 255.0f,
                                                             color_image_data[4 * i + 0] / 255.0f));
                } else {
                    cloud->points_.push_back(Eigen::Vector3d(0, 0, 0));
                    cloud->colors_.push_back(Eigen::Vector3d(0, 0, 0));
                }
            }

            Eigen::Matrix3d rotation_matrix;
            rotation_matrix << cas::eulerAngle2RotationMatrix(cur_angle);
            //定义个绕y轴旋转90度的旋转矩阵
//                rotation_matrix << 0, 0, 1,
//                        0, 1, 0,
//                        -1, 0, 0;

            if (flag == 0) {
                flag = 1;
            } else {
                Eigen::Vector3d center = Eigen::Vector3d::Zero();
                cloud->Rotate(rotation_matrix, center);
            }
            cloud = cloud->VoxelDownSample(0.01);
            *final_cloud += *cloud;

            task_count--;
            cout << "task_count: " << task_count << endl;
            if (ready_to_break && task_count <= 0) {
                need_break = true;
            }

//            need_cal_cloud = false;

            //使用RemoveStatisticalOutliers函数去除离群点
            int nb_neighbors = 20;
            double std_ratio = 2.0;

            //======
//            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud <pcl::PointXYZRGB>);
//            pcl::VoxelGrid <pcl::PointXYZRGB> sor;
//            sor.setInputCloud(cloud);
//            sor.setLeafSize(0.01f, 0.01f, 0.01f);
//            sor.filter(*cloud_filtered);
            //=======

            //==========================================
            //先把点云数据序列化成字节数组
//            std::ofstream ofs("pcd_oarchive/" + filename_pc + ".pcd");
//            boost::archive::text_oarchive oa(ofs);
//            oa << *cloud_filtered;
//            ofs.close();

//            std::ifstream ifs("pcd/" + filename_pc + ".pcd");
//            std::stringstream ss;
//            ss << ifs.rdbuf();
//            ifs.close();
//            std::string serialized_data = ss.str();
//            webSocket.sendPointCloud(serialized_data);
            //==========================================

        }
    });

    // 获取IMU数据
    while (true) {
        if (device.get_imu_sample(&imu_sample, std::chrono::milliseconds(0))) { //获取当前的IMU数据
            float gx = imu_sample.gyro_sample.xyz.x;
            float gy = imu_sample.gyro_sample.xyz.y;
            float gz = imu_sample.gyro_sample.xyz.z;

            float dtf = (float) (imu_sample.acc_timestamp_usec - temp) / 1000000;
            if (temp == 0) {
                temp = (float) imu_sample.acc_timestamp_usec;
                continue;
            }
            temp = (float) imu_sample.acc_timestamp_usec;

            //计算欧拉角，先绕x轴旋转roll，再绕y轴旋转pitch，最后绕z轴旋转yaw
            prev_angle = calculateOrientation(gx, gy, gz, dtf, prev_angle);

            if (abs(prev_angle.yaw - prev_yaw) > 0.785) {
//                prev_roll = prev_angle.roll;
//                prev_pitch = prev_angle.pitch;
                prev_yaw = prev_angle.yaw;
                printf("saving...\n");
                // 修改共享变量
                task_count++;
//                need_cal_cloud = true;
                cv.notify_one();
            }
        }

        //如果旋转角度大于2π，就退出
        if (abs(prev_angle.yaw) > 3.2) {
            cout << "旋转角度大于π，退出" << endl;
            ready_to_break = true;
            //TODO: 消除冗余点
            //使用RemoveStatisticalOutliers函数去除离群点
//            auto result = final_cloud->RemoveStatisticalOutliers(nb_neighbors, std_ratio);
//            auto filtered_cloud = std::get<0>(result);

            cout << "等待线程结束..." << endl;
            while (true) {
                if (need_break) {
                    break;
                }
            }
            open3d::io::WritePointCloud("ply/final_cloud.ply", *final_cloud);
            break;
        }
    }

    // 释放，关闭设备
    rgb_image.reset();
    depth_image.reset();
    capture.reset();
    device.close();

    return 1;
}