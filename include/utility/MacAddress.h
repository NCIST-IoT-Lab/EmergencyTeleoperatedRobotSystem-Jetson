//
// Created by HoChihchou on 4/23/23.
//

#ifndef _MACADDRESS_H_
#define _MACADDRESS_H_

#include <iostream>
#include <regex>
#include <string>

using namespace std;

namespace etrs::utility {
    class MacAddress {
    public:
        static bool isValidMacAddress(const string &mac_address);
    };

} // namespace etrs::utility

#endif // _MACADDRESS_H_
