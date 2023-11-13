//
// Create by HoChihchou on 2023/3/27
//

#include "utility/Debug.h"

using namespace std;
using namespace etrs::utility;

DebugMessages::DebugMessages() {
    debug_messages = unordered_map<string, string>();
}

DebugMessages::DebugMessages(const unordered_map<string, string> &messages) {
    debug_messages = messages;
}

string& DebugMessages::operator[](const string key) {
    return debug_messages[key];
}

void DebugMessages::setMessages(const DebugMessages &messages) {
    debug_messages = messages.debug_messages;
}

void DebugMessages::updateMessages(const DebugMessages &messages) {
    for (auto &msg: messages.debug_messages) {
        debug_messages[msg.first] = msg.second;
    }
}