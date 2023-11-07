//
// Created by HoChihchou on 2023/11/06
//

#ifndef _MATH_UTILS_H_
#define _MATH_UTILS_H_

#include "Utility.h"

#include <cmath>
#include <iostream>

using namespace std;

namespace etrs::utility {
    class MathUtils {
    public:
        static float DegreeToRadian(float degree);

        static float RadianToDegree(float radian);
    };
} // namespace etrs::utility

#endif // _MATH_UTILS_H_
