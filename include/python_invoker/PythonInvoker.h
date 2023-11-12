//
// Created by HoChihchou on 2023/11/09
//

#ifndef _PYTHON_INVOKER_H_
#define _PYTHON_INVOKER_H_

#include <TypeConverters.h>
#include <Utility.h>
#include <boost/python.hpp>

// 基于Boost.Python的Python调用器
namespace etrs::py {
    class PythonInvoker {
    private:
        bool is_init;

    public:
        explicit PythonInvoker();

        ~PythonInvoker();

        // 初始化Python环境
        bool initailize();

        // 关闭Python环境
        void finalize();

        // 将Python模块导入
        void importModule(string path);

        // DetectionResultType detectObjects(string module, string func, PointCloudType point_cloud);

        template <typename T, typename U>
        T runFuncWithOneArg(string module, string func, U arg) {
            T result;
            try {
                bstpy::object py_module(bstpy::import(module.c_str()));
                bstpy::object p = ToPy<U>(arg);
                bstpy::object py_result(py_module.attr(func.c_str())(p));
                result = ToCpp<T>(py_result);
            } catch (...) {
                PyErr_Print();
                PyErr_Clear();
            }
            return result;
        }
    };

} // namespace etrs::py

#endif // _PYTHON_INVOKER_H_
