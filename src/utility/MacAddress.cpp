//
// Create by HoChihchou on 2023/3/27
//

#include "MacAddress.h"

using namespace std;
using namespace etrs::utility;

bool MacAddress::isValidMacAddress(const string &mac_address) {
    regex pattern("^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$"); // 正则表达式
    return regex_match(mac_address, pattern);
}