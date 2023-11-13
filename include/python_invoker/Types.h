//
// Created by HoChihchou on 2023/11/09
//

#ifndef _TYPES_H_
#define _TYPES_H_

#include <iostream>

#include <Eigen/Core>

using namespace std;

struct BoundingBoxType {
    float x;
    float y;
    float z;
    float l;
    float w;
    float h;
    float yaw;

    BoundingBoxType() {
        x = 0;
        y = 0;
        z = 0;
        l = 0;
        w = 0;
        h = 0;
        yaw = 0;
    }

    BoundingBoxType(float x, float y, float z, float l, float w, float h, float yaw) {
        this->x = x;
        this->y = y;
        this->z = z;
        this->l = l;
        this->w = w;
        this->h = h;
        this->yaw = yaw;
    }
};

struct DetectionObjectType {
    string label;
    BoundingBoxType bbox;
    float score;

    DetectionObjectType() {
        label = "";
        bbox = BoundingBoxType();
        score = 0;
    }

    DetectionObjectType(string label, BoundingBoxType bbox, float score) {
        this->label = label;
        this->bbox = bbox;
        this->score = score;
    }
};

using PointType = Eigen::Vector3d;
using PointCloudType = vector<PointType>;
using DetectionResultType = vector<DetectionObjectType>;

#endif // _TYPES_H_