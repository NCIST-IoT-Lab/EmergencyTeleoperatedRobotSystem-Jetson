//
// Created by HoChihchou on 2023/11/12.
//

#include <iostream>

#include "geometry/Mesh.h"

using namespace etrs;
namespace etrs_geo = etrs::geometry;
namespace o3d_geo = open3d::geometry;

template <typename T>
void etrs_geo::Mesh::RotateMesh(T &point_cloud, etrs_geo::Axis axis, float angle) {
    Debug::CoutError("RotateMesh: 模板T类型错误，只能为open3d::geometry::TriangleMesh或t::geometry::TriangleMesh");
}

template <>
void etrs_geo::Mesh::RotateMesh<o3d_geo::TriangleMesh>(o3d_geo::TriangleMesh &point_cloud,
                                                                etrs_geo::Axis axis, float angle) {
    if (axis == etrs_geo::Axis::X) {
        point_cloud.Rotate(etrs_geo::Rotation::GetRotationMatrixX<Eigen::Matrix3d>(angle), Eigen::Vector3d::Zero());
    } else if (axis == etrs_geo::Axis::Y) {
        point_cloud.Rotate(etrs_geo::Rotation::GetRotationMatrixY<Eigen::Matrix3d>(angle), Eigen::Vector3d::Zero());
    } else if (axis == etrs_geo::Axis::Z) {
        point_cloud.Rotate(etrs_geo::Rotation::GetRotationMatrixZ<Eigen::Matrix3d>(angle), Eigen::Vector3d::Zero());
    }
}
template <>
void etrs_geo::Mesh::RotateMesh<t::geometry::TriangleMesh>(t::geometry::TriangleMesh &point_cloud, etrs_geo::Axis axis,
                                                           float angle) {
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

// 显式实例化模板函数
template void etrs_geo::Mesh::RotateMesh(o3d_geo::TriangleMesh &, etrs_geo::Axis, float);
template void etrs_geo::Mesh::RotateMesh(t::geometry::TriangleMesh &, etrs_geo::Axis, float);