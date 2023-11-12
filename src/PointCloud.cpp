//
// Created by root on 4/5/23.
//

#include "PointCloud.h"

#include <iostream>

using namespace etrs;

// void pcd::PointCloud::PreProcessPointCloud() {}

// void pcd::PointCloud::RotatePointCloud(geometry::PointCloud &point_cloud, pcd::Axis axis, float angle) {

template <typename T>
void pcd::PointCloud::DownSamplePointCloud(T &point_cloud, float voxel_size) {
    if (voxel_size <= 0) {
        Debug::CoutError("DownSamplePointCloud: voxel_size 必须大于 0");
        return;
    }
    point_cloud.VoxelDownSample(voxel_size);
}

template <typename T>
void pcd::PointCloud::RotatePointCloud(T &point_cloud, pcd::Axis axis, float angle) {
    if (axis == pcd::Axis::X) {
        // point_cloud.Rotate(GetRotationMatrixX<Eigen::Matrix3d>(angle), Eigen::Vector3d(0, 0, 0));
    } else if (axis == pcd::Axis::Y) {
        // point_cloud.Rotate(GetRotationMatrixY<Eigen::Matrix3d>(angle), Eigen::Vector3d(0, 0, 0));
    } else if (axis == pcd::Axis::Z) {
        // point_cloud.Rotate(GetRotationMatrixZ<Eigen::Matrix3d>(angle), Eigen::Vector3d(0, 0, 0));
    }
}

template <>
void pcd::PointCloud::RotatePointCloud<geometry::PointCloud>(geometry::PointCloud &point_cloud, pcd::Axis axis,
                                                             float angle) {
    if (axis == pcd::Axis::X) {
        point_cloud.Rotate(GetRotationMatrixX<Eigen::Matrix3d>(angle), Eigen::Vector3d(0, 0, 0));
    } else if (axis == pcd::Axis::Y) {
        point_cloud.Rotate(GetRotationMatrixY<Eigen::Matrix3d>(angle), Eigen::Vector3d(0, 0, 0));
    } else if (axis == pcd::Axis::Z) {
        point_cloud.Rotate(GetRotationMatrixZ<Eigen::Matrix3d>(angle), Eigen::Vector3d(0, 0, 0));
    }
}
template <>
void pcd::PointCloud::RotatePointCloud<t::geometry::PointCloud>(t::geometry::PointCloud &point_cloud, pcd::Axis axis,
                                                                float angle) {
    if (axis == pcd::Axis::X) {
        point_cloud.Rotate(GetRotationMatrixX<core::Tensor>(angle),
                           core::Tensor::Zeros({3}, core::Dtype::Float64, core::Device("CPU:0")));
    } else if (axis == pcd::Axis::Y) {
        point_cloud.Rotate(GetRotationMatrixY<core::Tensor>(angle),
                           core::Tensor::Zeros({3}, core::Dtype::Float64, core::Device("CPU:0")));
    } else if (axis == pcd::Axis::Z) {
        point_cloud.Rotate(GetRotationMatrixZ<core::Tensor>(angle),
                           core::Tensor::Zeros({3}, core::Dtype::Float64, core::Device("CPU:0")));
    }
}

template <typename T>
T pcd::PointCloud::GetRotationMatrix(float angle, Eigen::Vector3d axis_vector) {
    auto matrix = Eigen::AngleAxisd(MathUtils::DegreeToRadian(angle), Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
    if constexpr (std::is_same<T, Eigen::Matrix3d>::value) { // 判断T是否为Eigen::Matrix3d类型
        return matrix;
    } else if constexpr (std::is_same<T, core::Tensor>::value) { // 判断T是否为core::Tensor类型
        return core::eigen_converter::EigenMatrixToTensor(matrix);
    }
    Debug::CoutError("GetRotationMatrix: 模板T类型错误，只能为Eigen::Matrix3d或core::Tensor");
    return;
}

template <>
Eigen::Matrix3d pcd::PointCloud::GetRotationMatrix(float angle, Eigen::Vector3d axis_vector) {
    return Eigen::AngleAxisd(MathUtils::DegreeToRadian(angle), Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
}

template <>
core::Tensor pcd::PointCloud::GetRotationMatrix(float angle, Eigen::Vector3d axis_vector) {
    return core::eigen_converter::EigenMatrixToTensor(
        Eigen::AngleAxisd(MathUtils::DegreeToRadian(angle), Eigen::Vector3d(1, 0, 0)).toRotationMatrix());
}

// 绕X轴旋转
template <typename T>
T pcd::PointCloud::GetRotationMatrixX(float angle) {
    return GetRotationMatrix<T>(angle, Eigen::Vector3d(1, 0, 0));
}

// 绕Y轴旋转
template <typename T>
T pcd::PointCloud::GetRotationMatrixY(float angle) {
    return GetRotationMatrix<T>(angle, Eigen::Vector3d(0, 1, 0));
}

// 绕Z轴旋转
template <typename T>
T pcd::PointCloud::GetRotationMatrixZ(float angle) {
    return GetRotationMatrix<T>(angle, Eigen::Vector3d(0, 0, 1));
}

template <typename T>
void pcd::PointCloud::AdjustPointCloudNum(T &point_cloud, int num_multiple) {
    if (num_multiple <= 0) {
        Debug::CoutError("AdjustPointCloudNum: num_multiple 必须大于 0");
        return;
    }
    // int point_count = point_cloud.points_.size();
    // TODO: 待确认Python推理的具体约束
}

// 显式实例化模板函数
template void pcd::PointCloud::DownSamplePointCloud(geometry::PointCloud &, float);
template void pcd::PointCloud::DownSamplePointCloud(t::geometry::PointCloud &, float);
template void pcd::PointCloud::RotatePointCloud(geometry::PointCloud &, pcd::Axis, float);
template void pcd::PointCloud::RotatePointCloud(t::geometry::PointCloud &, pcd::Axis, float);
template Eigen::Matrix3d pcd::PointCloud::GetRotationMatrixX<Eigen::Matrix3d>(float);
template core::Tensor pcd::PointCloud::GetRotationMatrixX<core::Tensor>(float);
template Eigen::Matrix3d pcd::PointCloud::GetRotationMatrixY<Eigen::Matrix3d>(float);
template core::Tensor pcd::PointCloud::GetRotationMatrixY<core::Tensor>(float);
template Eigen::Matrix3d pcd::PointCloud::GetRotationMatrixZ<Eigen::Matrix3d>(float);
template core::Tensor pcd::PointCloud::GetRotationMatrixZ<core::Tensor>(float);
template void pcd::PointCloud::AdjustPointCloudNum<geometry::PointCloud>(geometry::PointCloud &, int);
template void pcd::PointCloud::AdjustPointCloudNum<t::geometry::PointCloud>(t::geometry::PointCloud &, int);