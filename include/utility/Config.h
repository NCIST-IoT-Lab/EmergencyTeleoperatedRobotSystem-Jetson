//
// Created by HoChihchou on 4/23/23.
//

#ifndef _CONFIG_H_
#define _CONFIG_H_

// #include <chrono>
#include <cstdarg>
#include <fmt/format.h>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <sstream>
#include <string>

using namespace std;

#define COMMENT_CHAR '#'

namespace etrs::utility {
    // 读取配置
    class Config {
    private:
        string config_path;
        unordered_map<string, string> config_map;

    public:
        explicit Config();

        explicit Config(const string config_path, const char comment_char = COMMENT_CHAR);

        // TODO: 改重载[]运算符
        string get(const string key);

        float getFloat(const string key);

        int getInt(const string key);

        bool getBool(const string key);

        bool set(const string key, const string value);

    private:
        void tirm(string &str);
    };

} // namespace etrs::utility

#endif //_CONFIG_H_
