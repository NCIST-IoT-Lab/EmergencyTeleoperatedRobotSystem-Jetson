//
// Create by HoChihchou on 2023/11/07
//

#include "utility/MathUtils.h"

using namespace std;
using namespace etrs::utility;

float MathUtils::DegreeToRadian(float degree) {
    if (degree > 360.0 || degree < -360.0) {
        Debug::CoutWarning("DegreeToRadian: 角度超出范围[-360, 360]");
    }
    return degree / 180.0 * M_PI;
}

float MathUtils::RadianToDegree(float radian) {
    if (radian > 2 * M_PI || radian < -2 * M_PI) {
        Debug::CoutWarning("RadianToDegree: 弧度超出范围[-2π, 2π]");
    }
    return radian / M_PI * 180.0;
}