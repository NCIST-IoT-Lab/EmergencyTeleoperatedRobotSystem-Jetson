//
// Created by HoChihchou on 2023/11/06
//

#include "ObjectDetection.h"

using namespace std;
using namespace etrs::det3d;

ObjectDetector::ObjectDetector(): py_invoker(){
    py_invoker.initailize();
    py_invoker.importModule(
        "/home/ncistwlwsys/hezhizhou-projects/disk/SingleAzureKinect3DReconstruction/src/object_detection");
}

ObjectDetector::~ObjectDetector() {
    py_invoker.finalize();
}

DetectionResultType ObjectDetector::detectObjects(const PointCloudType &point_cloud) {
    return py_invoker.runFuncWithOneArg<DetectionResultType, PointCloudType>("object_detection", "detect_objects", point_cloud);
}



