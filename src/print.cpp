#include <iostream>
#include <string>

#include <open3d/Open3D.h>
#include <Utility.h>

using namespace std;
using namespace open3d;
using namespace etrs::utility;

int main(int argc, char **argv) {
    Debug::CoutSuccess("Azure Kinect 初始化成功！");
    Debug::CoutSuccess("机械臂初始化成功！");
    Debug::CoutSuccess("夹爪初始化成功！");
    Debug::CoutSuccess("底盘车初始化成功！");
    Debug::CoutDebug("尝试开启服务器...");
    Debug::CoutSuccess("服务器创建成功！");
    Debug::CoutDebug("等待客户端连接...");
    Debug::CoutSuccess("客户端连接成功");
    Debug::CoutSuccess("接收到机械臂数据");
    Debug::CoutInfo("机械臂数据：FF FE 0A 0C 02 00  03 02 02 04 FE");
    Debug::CoutSuccess("接收到底盘车数据");
    Debug::CoutInfo("底盘车数据：[1, 23.5, 90, 59, 34]");
    Debug::CoutSuccess("接收到夹爪数据");
    Debug::CoutInfo("夹爪数据：true");
}
