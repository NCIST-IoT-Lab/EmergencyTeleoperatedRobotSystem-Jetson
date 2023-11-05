//
// Created by HoChihchou on 4/23/23.
//

#ifndef _TIME_H_
#define _TIME_H_

#include <fmt/format.h>
#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>
#include <string>
#include <ctime>

using namespace std;

namespace etrs::utility {
    class Timer {
    public:
        // 获取当前时间戳
        static int64_t getCurrentTimeStamp();
        // 获取当前时间
        static string getCurrentTime(string fmt = "%y-%m-%d_%H-%M-%S");
    };

} // namespace etrs::utility

#endif // _TIME_H_
