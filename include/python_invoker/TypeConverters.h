//
// Created by HoChihchou on 2023/11/09
//

#ifndef _TYPE_CONVERTER_H_
#define _TYPE_CONVERTER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Utility.h>
#include <Types.h>
#include <boost/python.hpp>

using namespace std;
namespace bstpy = boost::python;

namespace etrs::py {
    // 将C++类型转换为Python类型
    template <class T>
    class ToPy {
    public:
        bstpy::object obj;

        ToPy(T data) { obj = bstpy::object(data); }

        operator bstpy::object() { return obj; }
    };

    template <>
    class ToPy<PointCloudType> {
    public:
        bstpy::object obj;

        ToPy(PointCloudType data) {
            bstpy::list py_list;
            for (auto item : data) {
                bstpy::list py_item;
                py_item.append(item[0]);
                py_item.append(item[1]);
                py_item.append(item[2]);
                py_list.append(py_item);
            }
            obj = py_list;
        }

        operator bstpy::object() { return obj; }
    };

    // 将Python类型转换为C++类型
    template <class T>
    class ToCpp {
    public:
        bstpy::object obj;

        ToCpp(bstpy::object data) : obj(data) {}

        operator T() {
            T value = bstpy::extract<T>(obj);
            return value;
        }
    };

    template <>
    class ToCpp<DetectionResultType> {
    public:
        bstpy::object obj;

        ToCpp(bstpy::object data) : obj(data) {}

        operator DetectionResultType() {
            DetectionResultType res_vector;
            bstpy::list py_list = bstpy::extract<bstpy::list>(obj);
            for (int i = 0; i < bstpy::len(py_list); i++) {
                bstpy::tuple py_tuple = bstpy::extract<bstpy::tuple>(py_list[i]);
                DetectionObjectType det_object;
                det_object.label = bstpy::extract<string>(py_tuple[0]);
                bstpy::tuple bboxes_tuple = bstpy::extract<bstpy::tuple>(py_tuple[1]);
                det_object.bbox = BoundingBoxType(float(bstpy::extract<float>(bboxes_tuple[0])),
                                                  float(bstpy::extract<float>(bboxes_tuple[1])),
                                                  float(bstpy::extract<float>(bboxes_tuple[2])),
                                                  float(bstpy::extract<float>(bboxes_tuple[3])),
                                                  float(bstpy::extract<float>(bboxes_tuple[4])),
                                                  float(bstpy::extract<float>(bboxes_tuple[5])),
                                                  float(bstpy::extract<float>(bboxes_tuple[6])));
                det_object.score = bstpy::extract<float>(py_tuple[2]);
                res_vector.push_back(det_object);
            }
            return res_vector;
        }
    };

} // namespace etrs::py

#endif // _TYPE_CONVERTER_H_
