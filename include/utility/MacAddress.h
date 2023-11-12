//
// Created by HoChihchou on 4/23/23.
//

#ifndef _MAC_ADDRESS_H_
#define _MAC_ADDRESS_H_

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

#endif // _MAC_ADDRESS_H_
