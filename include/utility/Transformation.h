// //
// // Created by HoChihchou on 4/23/23.
// //

// #ifndef _TRANSFORMATION_H_
// #define _TRANSFORMATION_H_

// #include <cmath>
// #include <cstdarg>
// #include <iostream>
// #include <mutex>
// #include <string>

// #include <Eigen/Core>
// #include <Eigen/Dense>

// #include <open3d/Open3D.h>

// using namespace std;
// using namespace open3d;

// namespace etrs::utility {
//     class Transformation {
//     public:
//         static double CalculateTranslationNormT(const core::Tensor &transformation);

//         static Eigen::Matrix4d RemoveXZRotation(const Eigen::Matrix4d &transformation, const string direction);

//         static core::Tensor RemoveXZRotationT(const core::Tensor &transformation, const string direction);

//         static Eigen::Matrix4d RemoveYTranslation(const Eigen::Matrix4d &transformation);

//         static core::Tensor RemoveYTranslationT(const core::Tensor &transformation);
//     };
// } // namespace etrs::utility

// #endif // _TRANSFORMATION_H_
