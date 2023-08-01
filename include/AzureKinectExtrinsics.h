//
// Created by HoChihChou on 4/7/23.
//

#ifndef _CASAZUREKINECTEXTRINSICS_H_
#define _CASAZUREKINECTEXTRINSICS_H_

//导入Kinect库
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>

#include <Eigen/Core>
#include <Utility.h>

using namespace std;
using namespace etrs::utility;

//打开 Azure Kinect 设备
namespace etrs{
    // C++:
    //    k4a::device openAzureKinectDevice();
    bool openAzureKinectDevice(k4a_device_t *device);

    //void configureAzureKinectDevice(k4a::device &device, k4a_device_configuration_t config);
    bool configureAzureKinectDevice(k4a_device_t device, k4a_device_configuration_t config);

    bool getAzureKinectCalibration(k4a_device_t device, k4a_device_configuration_t config, k4a_calibration_t *calibration);

    bool getAzureKinectCapture(k4a_device_t device, k4a_capture_t *capture, int32_t timeout_in_ms);

    bool getAzureKinectDepthImage(k4a_capture_t capture, k4a_image_t *depth_image);

    bool getAzureKinectColorImage(k4a_capture_t capture, k4a_image_t *color_image);

    bool startAzureKinectImu(k4a_device_t device);

    bool getAzureKinectImuSample(k4a_device_t device, k4a_imu_sample_t *imu_sample, int32_t timeout_in_ms);

    class EulerAngle {
    public:
        double roll, pitch, yaw;                             //横滚角，俯仰角，偏航角，单位弧度
        EulerAngle(double r = 0, double p = 0, double y = 0);//构造函数
        // 算术运算符重载 = 
        EulerAngle &operator=(const double v);
    };

    // 计算角度
    EulerAngle calculateOrientation(double gx, double gy, double gz, double dt, const EulerAngle &prevAngle);

    // 角度转旋转矩阵
    Eigen::Matrix3d eulerAngle2RotationMatrix(const EulerAngle &angle);
}// namespace etrs


#endif//_CASAZUREKINECTEXTRINSICS_H_
