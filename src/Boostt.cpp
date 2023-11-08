#include <iostream>
#include <string>

#include <boost/python.hpp>

using namespace std;

int main(int argc, char **argv) {

    Py_Initialize();    // 初始化Python环境，分配Python使用资源
    if (!Py_IsInitialized()){
        return false;
    }

    try {
        PyRun_SimpleString("import sys");
        PyRun_SimpleString("sys.stdout.write('Hello, MyFriend\n')");
    } catch(...) {
        PyErr_Print();
        PyErr_Clear();
        Py_Finalize();
        return 0;
    }
    Py_Finalize();  // 关闭Python环境，释放资源
    return 0;
}

 