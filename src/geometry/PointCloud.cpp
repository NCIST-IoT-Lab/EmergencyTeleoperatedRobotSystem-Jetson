//
// Created by HoChihchou on 2023/11/06.
//

#include <iostream>

#include "geometry/PointCloud.h"

using namespace etrs;
namespace etrs_geo = etrs::geometry;
namespace o3d_geo = open3d::geometry;

template <typename T>
void etrs_geo::PointCloud::DownSamplePointCloud(T &point_cloud, float voxel_size) {
    if (voxel_size <= 0) {
        Debug::CoutError("DownSamplePointCloud: voxel_size 必须大于 0");
        return;
    }
    point_cloud.VoxelDownSample(voxel_size);
}

template <typename T>
void etrs_geo::PointCloud::RotatePointCloud(T &point_cloud, etrs_geo::Axis axis, float angle) {
    Debug::CoutError("RotatePointCloud: 模板T类型错误，只能为open3d::geometry::PointCloud或t::geometry::PointCloud");
}

template <>
void etrs_geo::PointCloud::RotatePointCloud<o3d_geo::PointCloud>(o3d_geo::PointCloud &point_cloud, etrs_geo::Axis axis,
                                                                 float angle) {
    if (axis == etrs_geo::Axis::X) {
        point_cloud.Rotate(etrs_geo::Rotation::GetRotationMatrixX<Eigen::Matrix3d>(angle), Eigen::Vector3d(0, 0, 0));
    } else if (axis == etrs_geo::Axis::Y) {
        point_cloud.Rotate(etrs_geo::Rotation::GetRotationMatrixY<Eigen::Matrix3d>(angle), Eigen::Vector3d(0, 0, 0));
    } else if (axis == etrs_geo::Axis::Z) {
        point_cloud.Rotate(etrs_geo::Rotation::GetRotationMatrixZ<Eigen::Matrix3d>(angle), Eigen::Vector3d(0, 0, 0));
    }
}

template <>
void etrs_geo::PointCloud::RotatePointCloud<t::geometry::PointCloud>(t::geometry::PointCloud &point_cloud,
                                                                     etrs_geo::Axis axis, float angle) {
    if (axis == etrs_geo::Axis::X) {
        point_cloud.Rotate(etrs_geo::Rotation::GetRotationMatrixX<core::Tensor>(angle),
                           core::Tensor::Zeros({3}, core::Dtype::Float64, core::Device("CPU:0")));
    } else if (axis == etrs_geo::Axis::Y) {
        point_cloud.Rotate(etrs_geo::Rotation::GetRotationMatrixY<core::Tensor>(angle),
                           core::Tensor::Zeros({3}, core::Dtype::Float64, core::Device("CPU:0")));
    } else if (axis == etrs_geo::Axis::Z) {
        point_cloud.Rotate(etrs_geo::Rotation::GetRotationMatrixZ<core::Tensor>(angle),
                           core::Tensor::Zeros({3}, core::Dtype::Float64, core::Device("CPU:0")));
    }
}

template <typename T>
void etrs_geo::PointCloud::AdjustPointCloudNum(T &point_cloud, int num_multiple) {
    if (num_multiple <= 0) {
        Debug::CoutError("AdjustPointCloudNum: num_multiple 必须大于 0");
        return;
    }
    // int point_count = point_cloud.points_.size();
    // TODO: 待确认Python推理的具体约束
}

// 显式实例化模板函数
template void etrs_geo::PointCloud::DownSamplePointCloud<o3d_geo::PointCloud>(o3d_geo::PointCloud &, float);
template void etrs_geo::PointCloud::DownSamplePointCloud<t::geometry::PointCloud>(t::geometry::PointCloud &, float);
template void etrs_geo::PointCloud::RotatePointCloud<o3d_geo::PointCloud>(o3d_geo::PointCloud &, etrs_geo::Axis, float);
template void etrs_geo::PointCloud::RotatePointCloud<t::geometry::PointCloud>(t::geometry::PointCloud &, etrs_geo::Axis, float);
template void etrs_geo::PointCloud::AdjustPointCloudNum<o3d_geo::PointCloud>(o3d_geo::PointCloud &, int);
template void etrs_geo::PointCloud::AdjustPointCloudNum<t::geometry::PointCloud>(t::geometry::PointCloud &, int);