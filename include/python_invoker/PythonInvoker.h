//
// Created by HoChihchou on 2023/11/09
// 

#ifndef _PYTHON_INVOKER_H_
#define _PYTHON_INVOKER_H_

#include <Utility.h>
#include <TypeConverters.h>
#include <boost/python.hpp>

// 基于Boost.Python的Python调用器
namespace etrs::py {
    class PythonInvoker{
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

        // 运行无参数Python函数
        void runFunc(string module, string func);

        // 运行有参数Python函数
        // int runFunc(const string module, const string func, const vector<int> args);
        // template<typename T>
        // T add(string module, string func, vector<T> paras);
        
        DetectionResultType detectObjects(string module, string func, PointCloudType point_cloud);
    };

} // namespace etrs::py

#endif // _PYTHON_INVOKER_H_
