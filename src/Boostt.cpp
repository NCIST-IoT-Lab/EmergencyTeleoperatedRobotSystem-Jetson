#include <iostream>
// #include <python_invoker/PythonInvoker.h>
// #include <python_invoker/TypeConverters.h>
#include <string>
#include <boost/python.hpp>
#include <ObjectDetection.h>
#include <open3d/Open3D.h>

using namespace std;
using namespace etrs::py;

int main(int argc, char **argv) {
    // PythonInvoker py_invoker;
    // py_invoker.initailize();
    // py_invoker.importModule(
    //     "/home/ncistwlwsys/hezhizhou-projects/disk/SingleAzureKinect3DReconstruction/src/object_detection");

    open3d::geometry::PointCloud pcd;
    open3d::io::ReadPointCloud("ply/nice_point_cloud2.ply", pcd);
    PointCloudType point_cloud = pcd.points_;

    etrs::det3d::ObjectDetector object_detector;
    DetectionResultType result = object_detector.detectObjects(pcd.points_);

    // DetectionResultType result = py_invoker.detectObjects("object_detection", "detect_objects", point_cloud);
    // DetectionResultType result = py_invoker.runFuncWithOneArg<DetectionResultType, PointCloudType>("object_detection", "detect_objects", point_cloud);
    for (auto item : result) {
        cout << "==== 物体: "<< item.label << "=======" << endl;
        BoundingBoxType bbox = item.bbox;
        cout << item.bbox.x << " " << item.bbox.y << " " << item.bbox.z << " " << item.bbox.l << " " << item.bbox.w
             << " " << item.bbox.h << " " << item.bbox.yaw << endl;
        cout << item.score << endl;
    }
    return 0;
}
