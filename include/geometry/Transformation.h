//
// Created by HoChihchou on 2023/11/12
//

#ifndef _TRANSFORMATION_H_
#define _TRANSFORMATION_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <open3d/Open3D.h>

#include "utility/Utility.h"

using namespace std;
using namespace etrs::utility;
using namespace open3d;

namespace etrs::geometry {
    // 给RotatePointCloud的坐标轴
    enum Axis { X, Y, Z };

    class Rotation{
    public:
        // 绕任意轴旋转
        template <typename T>
        static T GetRotationMatrix(Eigen::Vector3d axis_vector, float angle);

        // 绕X轴旋转
        template <typename T>
        static T GetRotationMatrixX(float angle);

        // 绕Y轴旋转
        template <typename T>
        static T GetRotationMatrixY(float angle);

        // 绕Z轴旋转
        template <typename T>
        static T GetRotationMatrixZ(float angle);

        template <typename T>
        static T RemoveRotationXZ(const T &transformation, string direction);
    };

    class Translation{
    public:
        template <typename T>
        static T RemoveTranslationY(const T &transformation);

        static double CalculateTranslationNorm(const core::Tensor &transformation);
    }; 
} // namespace etrs::geometry

#endif //_TRANSFORMATION_H_
