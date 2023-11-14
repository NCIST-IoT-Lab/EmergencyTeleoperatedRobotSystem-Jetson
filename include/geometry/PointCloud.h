//
// Created by HoChihchou on 2023/11/06
//

#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <open3d/Open3D.h>

#include "python_invoker/Types.h"
#include "geometry/Transformation.h"
#include "utility/Utility.h"

using namespace std;
using namespace etrs::utility;
using namespace open3d;

namespace etrs::geometry {

    class PointCloud {
    public:
        // static void PreProcessPointCloud();

        // 下采样点云
        template <typename T>
        static void DownSamplePointCloud(T &point_cloud, float voxel_size);

        // 旋转点云
        template <typename T>
        static void RotatePointCloud(T &point_cloud, etrs::geometry::Axis axis, float angle);

        // 将点云数量微调到某个值的倍数（删点）
        template <typename T>
        static void AdjustPointCloudNum(T &point_cloud, int num_multiple);
    };

} // namespace etrs::geometry

#endif //_POINT_CLOUD_H_
