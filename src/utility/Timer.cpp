//
// Create by HoChihchou on 2023/3/27
//

#include "utility/Timer.h"

using namespace std;
using namespace etrs::utility;

// 获取当前时间戳
int64_t Timer::getCurrentTimeStamp() {
    auto now = chrono::system_clock::now();
    auto in_time_t = chrono::system_clock::to_time_t(now); // 获取当前时间转换为 time_t 类型
    auto seconds = chrono::duration_cast<chrono::seconds>(now.time_since_epoch());
    return seconds.count();
}

// 获取当前时间
string Timer::getCurrentTime(string fmt) {
    auto in_time_t = chrono::system_clock::to_time_t(chrono::system_clock::now()); // 获取当前时间转换为 time_t 类型
    stringstream ss;
    ss << put_time(localtime(&in_time_t), fmt.c_str());
    return ss.str();
}