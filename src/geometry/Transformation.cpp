//
// Created by HoChihchou on 2023/11/12.
//

#include <iostream>

#include "geometry/Transformation.h"

using namespace etrs;
namespace etrs_geo = etrs::geometry;

template <typename T>
T etrs_geo::Rotation::GetRotationMatrix(Eigen::Vector3d axis_vector, double angle) {
    // auto matrix = Eigen::AngleAxisd(MathUtils::DegreeToRadian(angle), Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
    // if constexpr (std::is_same<T, Eigen::Matrix3d>::value) { // 判断T是否为Eigen::Matrix3d类型
    //     return matrix;
    // } else if constexpr (std::is_same<T, core::Tensor>::value) { // 判断T是否为core::Tensor类型
    //     return core::eigen_converter::EigenMatrixToTensor(matrix);
    // }
    Debug::CoutError("GetRotationMatrix: 模板T类型错误，只能为Eigen::Matrix3d或core::Tensor");
    return;
}

template <>
Eigen::Matrix3d etrs_geo::Rotation::GetRotationMatrix(Eigen::Vector3d axis_vector, double angle) {
    return Eigen::AngleAxisd(MathUtils::DegreeToRadian(angle), axis_vector).toRotationMatrix();
}

template <>
core::Tensor etrs_geo::Rotation::GetRotationMatrix(Eigen::Vector3d axis_vector, double angle) {
    return core::eigen_converter::EigenMatrixToTensor(
        Eigen::AngleAxisd(MathUtils::DegreeToRadian(angle), axis_vector).toRotationMatrix());
}

// 绕X轴旋转
template <typename T>
T etrs_geo::Rotation::GetRotationMatrixX(double angle) {
    return GetRotationMatrix<T>(Eigen::Vector3d(1, 0, 0), angle);
}

// 绕Y轴旋转
template <typename T>
T etrs_geo::Rotation::GetRotationMatrixY(double angle) {
    return GetRotationMatrix<T>(Eigen::Vector3d(0, 1, 0), angle);
}

// 绕Z轴旋转
template <typename T>
T etrs_geo::Rotation::GetRotationMatrixZ(double angle) {
    return GetRotationMatrix<T>(Eigen::Vector3d(0, 0, 1), angle);
}

template <typename T>
T etrs_geo::Rotation::RemoveRotationXZ(const T &transformation, string direction) {
    Debug::CoutError("RemoveRotationXZ: 模板T类型错误，只能为Eigen::Matrix4d或core::Tensor");
    return;
}

template <>
Eigen::Matrix4d etrs_geo::Rotation::RemoveRotationXZ(const Eigen::Matrix4d &transformation, string direction) {
    Eigen::Vector3d euler_angles = transformation.block<3, 3>(0, 0).eulerAngles(2, 1, 0);

    double mod = fmod(M_PI, fabs(euler_angles[1]));
    // TODO: 删除参数 direction
    if (direction == "R") {
        mod = -mod;
    }
    Eigen::Matrix3d rotation_matrix;
    // TODO: 能否改成下列形式
    rotation_matrix = Eigen::AngleAxisd(mod, Eigen::Vector3d::UnitY());
    // Eigen::Matrix3d temp_rx(Eigen::AngleAxisd(mod, Eigen::Vector3d::UnitY()));
    // Debug::CoutDebug("orig_rx: {}, {}, {}", rotation_matrix(0), rotation_matrix(1), rotation_matrix(2));
    // Debug::CoutDebug("temp_rx: {}, {}, {}", temp_rx[0], temp_rx[1], temp_rx[2]);

    Eigen::Vector3d euler_angles1 = rotation_matrix.eulerAngles(2, 1, 0);
    // Debug::CoutDebug("NEW_EulerAngles: {}, {}, {}", euler_angles1[0], euler_angles1[1], euler_angles1[2]);

    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block<3, 3>(0, 0) = rotation_matrix;
    result.block<3, 1>(0, 3) = transformation.block<3, 1>(0, 3);

    return result;
}

template <>
core::Tensor etrs_geo::Rotation::RemoveRotationXZ(const core::Tensor &transformation, string direction) {
    // 转成Eigen
    Eigen::Matrix4d transformation_eigen = core::eigen_converter::TensorToEigenMatrixXd(transformation);
    // 调用RemoveXZRotation
    Eigen::Matrix4d result = RemoveRotationXZ(transformation_eigen, direction);
    // 转成Tensor
    return core::eigen_converter::EigenMatrixToTensor(result);
}

template <typename T>
T etrs_geo::Translation::RemoveTranslationY(const T &transformation) {
    Debug::CoutError("RemoveTranslationY: 模板T类型错误，只能为Eigen::Matrix4d或core::Tensor");
    return;
}

void etrs_geo::Rotation::RotateBoundingBoxes(DetectionResultType &result, etrs::geometry::Axis axis, double angle) {
    Eigen::Matrix3d R;
    if (axis == etrs_geo::Axis::X) {
        R = GetRotationMatrixX<Eigen::Matrix3d>(angle);
    } else if (axis == etrs_geo::Axis::Y) {
        R = GetRotationMatrixY<Eigen::Matrix3d>(angle);
    } else if (axis == etrs_geo::Axis::Z) {
        R = GetRotationMatrixZ<Eigen::Matrix3d>(angle);
    }
    for (auto &object : result) {
        Eigen::Map<Eigen::Vector3d> translation(&object.bbox.x);    // 通过引用传递，不做值传递和拷贝
        translation = R * translation;  //原点
    }
}

template <>
Eigen::Matrix4d etrs_geo::Translation::RemoveTranslationY(const Eigen::Matrix4d &transformation) {
    // Eigen::Matrix4d result = transformation;
    Eigen::Matrix4d result(transformation);
    result(1, 3) = 0;
    return result;
}

template <>
core::Tensor etrs_geo::Translation::RemoveTranslationY(const core::Tensor &transformation) {
    // core::Tensor result = transformation.Clone();
    core::Tensor result(transformation); // 使用移动构造函数
    result[1][3] = 0;
    return result;
}

double etrs_geo::Translation::CalculateTranslationNorm(const core::Tensor &transformation) {
    core::Tensor translation(transformation.Slice(0, 0, 3).Slice(1, 3, 4));
    return sqrt((translation * translation).Sum({0, 1}).Item<double>());
}

// 显式实例化模板函数
template Eigen::Matrix3d etrs_geo::Rotation::GetRotationMatrix(Eigen::Vector3d, double);
template core::Tensor etrs_geo::Rotation::GetRotationMatrix(Eigen::Vector3d, double);
template Eigen::Matrix3d etrs_geo::Rotation::GetRotationMatrixX<Eigen::Matrix3d>(double);
template core::Tensor etrs_geo::Rotation::GetRotationMatrixX<core::Tensor>(double);
template Eigen::Matrix3d etrs_geo::Rotation::GetRotationMatrixY<Eigen::Matrix3d>(double);
template core::Tensor etrs_geo::Rotation::GetRotationMatrixY<core::Tensor>(double);
template Eigen::Matrix3d etrs_geo::Rotation::GetRotationMatrixZ<Eigen::Matrix3d>(double);
template core::Tensor etrs_geo::Rotation::GetRotationMatrixZ<core::Tensor>(double);
template Eigen::Matrix4d etrs_geo::Rotation::RemoveRotationXZ<Eigen::Matrix4d>(const Eigen::Matrix4d &, string);
template core::Tensor etrs_geo::Rotation::RemoveRotationXZ<core::Tensor>(const core::Tensor &, string);
template Eigen::Matrix4d etrs_geo::Translation::RemoveTranslationY<Eigen::Matrix4d>(const Eigen::Matrix4d &);
template core::Tensor etrs_geo::Translation::RemoveTranslationY<core::Tensor>(const core::Tensor &);