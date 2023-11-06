//
// Created by HoChihchou on 4/23/23.
//

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <cstdarg>
#include <fmt/format.h>
#include <iostream>
#include <string>
#include <unordered_map>

using namespace std;

#define COUT_RESET "\033[0m"   // 清除颜色
#define COUT_RED "\033[31m"    // 红色
#define COUT_GREEN "\033[32m"  // 绿色
#define COUT_YELLOW "\033[33m" // 黄色
#define COUT_BLUE "\033[34m"   // 蓝色

// 加粗
#define COUT_BOLD "\033[1m"

// ETRS
#define COUT_MSG_BOLD_ETRS COUT_BOLD << "[ETRS] " << COUT_RESET
// 错误信息
#define COUT_MSG_BOLD_ETRS_ERROR COUT_RED << COUT_BOLD << "[ETRS ERROR] " << COUT_RESET
// 成功信息
#define COUT_MSG_BOLD_ETRS_SUCCESS COUT_GREEN << COUT_BOLD << "[ETRS SUCCESS] " << COUT_RESET
// 警告信息
#define COUT_MSG_BOLD_ETRS_WARNING COUT_YELLOW << COUT_BOLD << "[ETRS WARNING] " << COUT_RESET
// 调试信息
#define COUT_MSG_BOLD_ETRS_DEBUG COUT_BLUE << COUT_BOLD << "[ETRS DEBUG...] " << COUT_RESET

namespace etrs::utility {
    class DebugMessages {
    private:
        unordered_map<string, string> debug_messages;

    public:
        DebugMessages();

        // 允许隐式转换
        DebugMessages(const unordered_map<string, string> &messages);

        string &operator[](const string key);

        void setMessages(const DebugMessages &messages);

        void updateMessages(const DebugMessages &messages);

        // void set(const string key, const string value);
        // string get(const string key);
    };

    class Debug {
    public:
        template <typename T, typename... Args>
        static void CoutInfo(T format, Args... args) {
            cout << COUT_MSG_BOLD_ETRS << fmt::format(format, args...) << COUT_RESET << endl;
        }

        template <typename T, typename... Args>
        static void CoutError(T format, Args... args) {
            cout << COUT_MSG_BOLD_ETRS_ERROR << fmt::format(format, args...) << COUT_RESET << endl;
        }

        template <typename T, typename... Args>
        static void CoutSuccess(T format, Args... args) {
            cout << COUT_MSG_BOLD_ETRS_SUCCESS << fmt::format(format, args...) << COUT_RESET << endl;
        }

        template <typename T, typename... Args>
        static void CoutWarning(T format, Args... args) {
            cout << COUT_MSG_BOLD_ETRS_WARNING << fmt::format(format, args...) << COUT_RESET << endl;
        }

        template <typename T, typename... Args>
        static void CoutDebug(T format, Args... args) {
            cout << COUT_MSG_BOLD_ETRS_DEBUG << fmt::format(format, args...) << COUT_RESET << endl;
        }

        // 百分比变化，使用 std::flush;
        template <typename T, typename... Args>
        static void CoutFlush(T format, Args... args) {
            cout << '\r' << COUT_MSG_BOLD_ETRS << fmt::format(format, args...) << COUT_RESET << std::flush;
        }

        template <typename T, typename... Args>
        static void CoutSection(const string &sectionTitle, T format, Args... args) {
            const string line = string(10, '=');
            cout << line << endl;
            cout << sectionTitle << endl;
            cout << fmt::format(format, args...) << endl;
            cout << line << endl;
        }
    };

} // namespace etrs::utility

#endif // _DEBUG_H_
