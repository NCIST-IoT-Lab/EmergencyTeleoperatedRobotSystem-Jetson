//
// Created by HoChihchou on 2023/11/09
// 

#include "PythonInvoker.h"

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
        return false;
    }
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
        PyErr_Print();
        PyErr_Clear();
    }
}

void PythonInvoker::runFunc(string module, string func) {
    try {
        bstpy::object py_module(bstpy::import(module.c_str()));
        py_module.attr(func.c_str())();
    } catch(...) {
        PyErr_Print();
        PyErr_Clear();
    }
}

// int PythonInvoker::runFunc(const string module, const string func, const vector<int> args) {
    // int result = -1;
    // try {
    //     boost::python::object py_module(boost::python::import(module.c_str()));
    //     boost::python::object py_func = py_module.attr(func.c_str());
    //     boost::python::list py_args;
    //     for (auto arg : args) {
    //         py_args.append(arg);
    //     }
    //     result = boost::python::extract<int>(py_func(py_args));
    // } catch(...) {
    //     PyErr_Print();
    //     PyErr_Clear();
    // }
    // return result;
// }

DetectionResultType PythonInvoker::detectObjects(string module, string func, PointCloudType point_cloud) {
    DetectionResultType result;
    try {
        bstpy::object py_module(bstpy::import(module.c_str()));
        bstpy::object p = ToPy<PointCloudType>(point_cloud);
        bstpy::object py_result(py_module.attr(func.c_str())(p));
        result = ToCpp<DetectionResultType>(py_result);
    } catch(...) {
        PyErr_Print();
        PyErr_Clear();
    }
    return result;
}
