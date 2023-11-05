//
// Create by HoChihchou on 2023/3/27
//

#include "Transformation.h"
#include "Debug.h"

using namespace std;
using namespace etrs::utility;

double Transformation::CalculateTranslationNormT(const core::Tensor &transformation) {
    core::Tensor translation = transformation.Slice(0, 0, 3).Slice(1, 3, 4);
    // 打印
    // cout << translation[0].ToString() << ", " << translation[1].ToString() <<", "<< translation[2].ToString() <<
    // endl;
    return sqrt((translation * translation).Sum({0, 1}).Item<double>());
}

Eigen::Matrix4d Transformation::RemoveXZRotation(const Eigen::Matrix4d &transformation, const string direction) {
    Eigen::Matrix3d rotation = transformation.block<3, 3>(0, 0);
    Eigen::Vector3d translation = transformation.block<3, 1>(0, 3);
    Eigen::Vector3d euler_angles = rotation.eulerAngles(2, 1, 0); // ZYX
    Debug::CoutDebug("ZYX_EulerAngles: {}, {}, {}", euler_angles[0], euler_angles[1], euler_angles[2]);

    Eigen::Matrix3d rotation_matrix;
    double mod = fmod(M_PI, fabs(euler_angles[1]));
    if (direction == "R") {
        mod = -mod;
    }
    rotation_matrix = Eigen::AngleAxisd(mod, Eigen::Vector3d::UnitY());

    // rotation_matrix = Eigen::AngleAxisd(0.016, Eigen::Vector3d::UnitY());

    Eigen::Vector3d euler_angles1 = rotation_matrix.eulerAngles(2, 1, 0);
    Debug::CoutDebug("NEW_EulerAngles: {}, {}, {}", euler_angles1[0], euler_angles1[1], euler_angles1[2]);

    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block<3, 3>(0, 0) = rotation_matrix;
    result.block<3, 1>(0, 3) = translation;

    return result;
}

core::Tensor Transformation::RemoveXZRotationT(const core::Tensor &transformation, const string direction) {
    // 转成Eigen
    Eigen::Matrix4d transformation_eigen = core::eigen_converter::TensorToEigenMatrixXd(transformation);
    // 调用RemoveXZRotation
    Eigen::Matrix4d result = RemoveXZRotation(transformation_eigen, direction);
    // 转成Tensor
    return core::eigen_converter::EigenMatrixToTensor(result);
}

Eigen::Matrix4d Transformation::RemoveYTranslation(const Eigen::Matrix4d &transformation) {
    Eigen::Matrix4d result = transformation;
    result(1, 3) = 0;
    return result;
}

core::Tensor Transformation::RemoveYTranslationT(const core::Tensor &transformation) {
    core::Tensor result = transformation.Clone();
    // 打印 Tensor
    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 4; ++j) {
    //         cout << result[i][j].ToString() << " ";
    //     }
    //     cout << endl;
    // }
    result[1][3] = 0;
    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 4; ++j) {
    //         cout << result[i][j].ToString() << " ";
    //     }
    //     cout << endl;
    // }
    return result;
}