//
// Created by HoChihchou on 2023/11/12
//

#ifndef _MESH_H_
#define _MESH_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <open3d/Open3D.h>

#include <geometry/Transformation.h>
#include <Utility.h>


using namespace std;
using namespace etrs::utility;
using namespace open3d;

namespace etrs::geometry {
    class Mesh {
    public:
        template <typename T>
        static void RotateMesh(T &point_cloud, etrs::geometry::Axis axis, float angle);
    };

} // namespace etrs::geometry

#endif //_MESH_H_
