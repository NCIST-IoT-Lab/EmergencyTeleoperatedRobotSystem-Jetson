        //
// Created by Cassius0924 on 2023/11/06
//

#ifndef _OBJECT_DETECTION_H_
#define _OBJECT_DETECTION_H_

#include <PythonInvoker.h>
#include <DataMessage.pb.h>
#include <Utility.h>

using namespace etrs::utility;
using namespace etrs::py;

namespace etrs::det3d {
    class ObjectDetector {
    private:
        PythonInvoker py_invoker;
        
    public:
        explicit ObjectDetector();
        ~ObjectDetector();

        DetectionResultType detectObjects(const PointCloudType &point_cloud);
    };
} // namespace etrs::det3d

#endif // _OBJECT_DETECTION_H_
