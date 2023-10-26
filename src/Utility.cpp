//
// Create by HoChihchou on 2023/3/27
//
// 读取配置文件

#include "Utility.h"

using namespace std;
using namespace etrs::utility;

void tirm(string &str) {
    str.erase(0, str.find_first_not_of(" "));
    str.erase(str.find_last_not_of(" ") + 1);
}

Config::Config(const string config_path, const char comment_char) {
    this->config_path = config_path;
    ifstream in(this->config_path);
    if (in) {
        Debug::CoutSuccess("打开配置文件成功");
    } else {
        Debug::CoutError("{}，配置文件不存在", this->config_path);
        return;
    }
    string line;
    while (getline(in, line)) {
        if (line.empty() || line[0] == comment_char) { // 空行或注释行
            continue;
        }
        int pos = line.find("="); // 找到等号
        if (pos == -1) {
            continue; // 没有等号，跳过
        }
        string key = line.substr(0, pos);
        string value = line.substr(pos + 1);
        tirm(key);
        tirm(value);
        this->config_map[key] = value;
    }
}

string Config::get(const string key) {
    map<string, string>::iterator iter;
    iter = this->config_map.find(key);
    if (iter != this->config_map.end()) { // 找到了
        return iter->second;
    }
    Debug::CoutError("找不到配置项: {}，返回默认值 \"\"", key);
    return "";
}

float Config::getFloat(const string key) {
    map<string, string>::iterator iter;
    iter = this->config_map.find(key);
    if (iter != this->config_map.end()) {
        return stof(iter->second);
    }
    Debug::CoutError("找不到配置项: {}，返回默认值 0.0", key);
    return 0.0;
}

int Config::getInt(const string key) {
    map<string, string>::iterator iter;
    iter = this->config_map.find(key);
    if (iter != this->config_map.end()) {
        return stoi(iter->second);
    }
    Debug::CoutError("找不到配置项: {}，返回默认值 0", key);
    return 0;
}

bool Config::getBool(const string key) {
    map<string, string>::iterator iter;
    iter = this->config_map.find(key);
    if (iter != this->config_map.end()) {
        return iter->second == "true";
    }
    Debug::CoutError("找不到配置项: {}，返回默认值 false", key);
    return false;
}

// 找到配置项，并修改，写入文件
bool Config::set(const string key, const string value) {
    ifstream inputFile(this->config_path); // 打开配置文件进行读取
    string line;
    ostringstream modifiedContent;

    while (std::getline(inputFile, line)) {
        if (line.find(key) != std::string::npos) {               // 如果找到配置项
            modifiedContent << key << "=" << value << std::endl; // 修改值
        } else {
            modifiedContent << line << std::endl; // 保持原样
        }
    }

    inputFile.close();
    ofstream outputFile(this->config_path); // 打开配置文件进行写入
    outputFile << modifiedContent.str();    // 将修改后的内容写入文件
    outputFile.close();
    return true;
}

double Transformation::CalculateTranslationNormT(const core::Tensor &transformation) {
    core::Tensor translation = transformation.Slice(0, 0, 3).Slice(1, 3, 4);
    // 打印
    // cout << translation[0].ToString() << ", " << translation[1].ToString() <<", "<< translation[2].ToString() <<
    // endl;
    return sqrt((translation * translation).Sum({0, 1}).Item<double>());
}

Eigen::Matrix4d Transformation::RemoveXZRotation(const Eigen::Matrix4d &transformation, const string direction) {
    Eigen::Matrix3d rotation = transformation.block<3, 3>(0, 0);
    Eigen::Vector3d translation = transformation.block<3, 1>(0, 3);
    Eigen::Vector3d euler_angles = rotation.eulerAngles(2, 1, 0); // ZYX
    Debug::CoutDebug("ZYX_EulerAngles: {}, {}, {}", euler_angles[0], euler_angles[1], euler_angles[2]);

    Eigen::Matrix3d rotation_matrix;
    double mod = fmod(M_PI, fabs(euler_angles[1]));
    if (direction == "R") {
        mod = -mod;
    }
    rotation_matrix = Eigen::AngleAxisd(mod, Eigen::Vector3d::UnitY());

    // rotation_matrix = Eigen::AngleAxisd(0.016, Eigen::Vector3d::UnitY());

    Eigen::Vector3d euler_angles1 = rotation_matrix.eulerAngles(2, 1, 0);
    Debug::CoutDebug("NEW_EulerAngles: {}, {}, {}", euler_angles1[0], euler_angles1[1], euler_angles1[2]);

    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block<3, 3>(0, 0) = rotation_matrix;
    result.block<3, 1>(0, 3) = translation;

    return result;
}

core::Tensor Transformation::RemoveXZRotationT(const core::Tensor &transformation, const string direction) {
    // 转成Eigen
    Eigen::Matrix4d transformation_eigen = core::eigen_converter::TensorToEigenMatrixXd(transformation);
    // 调用RemoveXZRotation
    Eigen::Matrix4d result = RemoveXZRotation(transformation_eigen, direction);
    // 转成Tensor
    return core::eigen_converter::EigenMatrixToTensor(result);
}

Eigen::Matrix4d Transformation::RemoveYTranslation(const Eigen::Matrix4d &transformation) {
    Eigen::Matrix4d result = transformation;
    result(1, 3) = 0;
    return result;
}

core::Tensor Transformation::RemoveYTranslationT(const core::Tensor &transformation) {
    core::Tensor result = transformation.Clone();
    // 打印 Tensor
    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 4; ++j) {
    //         cout << result[i][j].ToString() << " ";
    //     }
    //     cout << endl;
    // }
    result[1][3] = 0;
    // for (int i = 0; i < 4; i++) {
    //     for (int j = 0; j < 4; ++j) {
    //         cout << result[i][j].ToString() << " ";
    //     }
    //     cout << endl;
    // }
    return result;
}

bool MacAddress::isValidMacAddress(const string &mac_address) {
    regex pattern("^([0-9A-Fa-f]{2}[:-]){5}([0-9A-Fa-f]{2})$"); // 正则表达式
    return regex_match(mac_address, pattern);
}

// 获取当前时间戳
int64_t Time::getCurrentTimeStamp() {
    auto now = chrono::system_clock::now();
    auto in_time_t = chrono::system_clock::to_time_t(now); // 获取当前时间转换为 time_t 类型
    auto seconds = chrono::duration_cast<chrono::seconds>(now.time_since_epoch());
    return seconds.count();
}

// 获取当前时间
string Time::getCurrentTime(string fmt) {
    auto in_time_t = chrono::system_clock::to_time_t(chrono::system_clock::now()); // 获取当前时间转换为 time_t 类型
    std::stringstream ss;
    ss << std::put_time(localtime(&in_time_t), fmt.c_str());
    return ss.str();
}