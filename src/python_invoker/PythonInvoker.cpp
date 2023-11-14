//
// Created by HoChihchou on 2023/11/09
// 

#include "python_invoker/PythonInvoker.h"

using namespace std;
using namespace etrs::utility;
using namespace etrs::py;

PythonInvoker::PythonInvoker(): is_init(false){ }

PythonInvoker::~PythonInvoker() { 
    finalize();
}

bool PythonInvoker::initailize() {
    if (is_init) {   
        return true;
    }
    Py_Initialize();    // 初始化Python环境，分配Python使用资源
    if (!Py_IsInitialized()){
        Debug::CoutError("三维目标检测环境初始化失败！(Python)");
        return false;
    }
    Debug::CoutSuccess("Python三维目标检测环境初始化成功！");
    is_init= true;
    return true;
}

void PythonInvoker::finalize() {
    if (!is_init) {
        return;
    }
    Py_Finalize();  // 关闭Python环境，释放资源
}

void PythonInvoker::importModule(string path) {
    try {
        PyRun_SimpleString("import sys");
        string cmd = "sys.path.append('" + path + "')";
        PyRun_SimpleString(cmd.c_str());
    } catch(...) {
        Debug::CoutError("Python模块导入失败！");
        PyErr_Print();
        PyErr_Clear();
    }
    Debug::CoutSuccess("Python模块：{} 导入成功！", path);
}

// DetectionResultType PythonInvoker::detectObjects(string module, string func, PointCloudType point_cloud) {
//     DetectionResultType result;
//     try {
//         bstpy::object py_module(bstpy::import(module.c_str()));
//         bstpy::object p = ToPy<PointCloudType>(point_cloud);
//         bstpy::object py_result(py_module.attr(func.c_str())(p));
//         result = ToCpp<DetectionResultType>(py_result);
//     } catch(...) {
//         PyErr_Print();
//         PyErr_Clear();
//     }
//     return result;
// }
