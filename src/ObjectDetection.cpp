//
// Created by Cassius0924 on 2023/11/06
//

#include "ObjectDetection.h"

using namespace std;
using namespace etrs::det3d;

ObjectDetector::ObjectDetector() {
}

ObjectDetector::~ObjectDetector() {
}

DetectionResultType ObjectDetector::detectObjects(const PointCloudType &point_cloud) {
    return py_invoker.runFuncWithOneArg<DetectionResultType, PointCloudType>("object_detection", "detect_objects", point_cloud);
}



