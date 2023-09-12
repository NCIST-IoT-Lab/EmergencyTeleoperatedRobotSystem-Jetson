//
// Created by HoChihchou on 4/23/23.
//

#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <cstdarg>
#include <fmt/format.h>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <open3d/Open3D.h>
#include <regex>
#include <sstream>
#include <string>

using namespace std;
using namespace open3d;

#define COMMENT_CHAR '#'

#define COUT_RESET "\033[0m"
#define COUT_RED "\033[31m"
#define COUT_GREEN "\033[32m"
#define COUT_YELLOW "\033[33m"
#define COUT_BLUE "\033[34m"

#define COUT_BOLD "\033[1m"

#define COUT_MSG_BOLD_ETRS COUT_BOLD << "[ETRS] " << COUT_RESET
#define COUT_MSG_BOLD_ETRS_ERROR COUT_RED << COUT_BOLD << "[ETRS ERROR] " << COUT_RESET
#define COUT_MSG_BOLD_ETRS_SUCCESS COUT_GREEN << COUT_BOLD << "[ETRS SUCCESS] " << COUT_RESET
#define COUT_MSG_BOLD_ETRS_WARNING COUT_YELLOW << COUT_BOLD << "[ETRS WARNING] " << COUT_RESET

namespace etrs::utility {
// 读取配置
class Config {
private:
    string config_path;
    map<string, string> config_map;

public:
    Config();

    Config(const string config_path, const char comment_char = COMMENT_CHAR);

    string get(const string key);

    float getFloat(const string key);

    int getInt(const string key);

    bool getBool(const string key);

    bool set(const string key, const string value);
};

class Debug {
public:
    template <typename... Args> static void CoutInfo(const char *format, Args... args) {
        cout << COUT_MSG_BOLD_ETRS << fmt::format(format, args...) << COUT_RESET << endl;
    }

    template <typename... Args> static void CoutError(const char *format, Args... args) {
        cout << COUT_MSG_BOLD_ETRS_ERROR << fmt::format(format, args...) << COUT_RESET << endl;
    }

    template <typename... Args> static void CoutSuccess(const char *format, Args... args) {
        cout << COUT_MSG_BOLD_ETRS_SUCCESS << fmt::format(format, args...) << COUT_RESET << endl;
    }

    template <typename... Args> static void CoutWarning(const char *format, Args... args) {
        cout << COUT_MSG_BOLD_ETRS_WARNING << fmt::format(format, args...) << COUT_RESET << endl;
    }

    template <typename... Args> static void CoutDebug(const char *format, Args... args) {
        cout << "[ETRS DEBUG...] " << fmt::format(format, args...) << COUT_RESET << endl;
    }

    // 百分比变化，使用 std::flush;
    template <typename... Args> static void CoutFlush(const char *format, Args... args) {
        cout << '\r' << COUT_MSG_BOLD_ETRS << fmt::format(format, args...) << COUT_RESET << std::flush;
    }

    template <typename... Args> static void CoutSection(const string &sectionTitle, const char *format, Args... args) {
        const std::string line = std::string(10, '=');
        cout << line << endl;
        cout << sectionTitle << endl;
        cout << fmt::format(format, args...) << endl;
        cout << line << endl;
    }
};

class Transformation {
public:
    static double CalculateTranslationNormT(const core::Tensor &transformation);

    static Eigen::Matrix4d RemoveXZRotation(const Eigen::Matrix4d &transformation, const string direction);

    static core::Tensor RemoveXZRotationT(const core::Tensor &transformation, const string direction);

    static Eigen::Matrix4d RemoveYTranslation(const Eigen::Matrix4d &transformation);

    static core::Tensor RemoveYTranslationT(const core::Tensor &transformation);
};

// 检查MAC地址是否合法
class MacAddress {
public:
    static bool isValidMacAddress(const string &mac_address);
};

} // namespace etrs::utility

#endif // _UTILITY_H_
