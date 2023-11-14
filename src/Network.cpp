#include "Network.h"

using namespace std;
using namespace etrs::net;

/*
 * 获取本地192.168开头的本地IP并打印
 * 参数：IP地址的存储地址（char*类型）
 * 返回值：获取成功返回0
 */
bool getLocalIp(char *ip) {
    int fd, intrface, retn = 0;        // fd是用户程序打开设备时使用open函数返回的文件标示符
    struct ifreq buf[INET_ADDRSTRLEN]; // INET_ADDRSTRLEN 宏定义，16
    struct ifconf ifc;                 // ifconf > ifreq
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) >=
        0) { // 创建套接字，存放AF_INET代表IPV4，SOCK_DGRAM代表创建的是数据报套接字/无连接的套接字，后面一般为0
        // 套接字创建成功
        ifc.ifc_len = sizeof(buf); // 所有网口加一起的长度 ifc.ifc_len 应该是一个出入参数
        // caddr_t,linux内核源码里定义的：typedef void *caddr_t；一般是一个int
        ifc.ifc_buf = (caddr_t)buf;                // 开辟网口缓冲区
        if (!ioctl(fd, SIOCGIFCONF, (char *)&ifc)) // linux系统调用函数，SIOCGIFCONF   获取所有接口(网口)的清单
        {
            intrface = ifc.ifc_len / sizeof(struct ifreq); // 获取到int值的网口总数
            while (intrface-- > 0) {
                if (!(ioctl(fd, SIOCGIFADDR, (char *)&buf[intrface]))) // SIOCGIFADDR 获取接口(网口)地址
                {
                    // inet_ntoa ()功能是将网络地址转换成“.”点隔的字符串格式。序列化
                    // 拿到该网口地址做对比
                    ip = (inet_ntoa(((struct sockaddr_in *)(&buf[intrface].ifr_addr))->sin_addr));
                    if (strstr(ip, "192.168.")) {
                        Debug::CoutInfo("服务器本地IP: {}", ip);
                        break;
                    }
                }
            }
        }
        close(fd);
        return true;
    }
    Debug::CoutError("获取本地IP地址失败");
    return false;
}

/**
 * BaseCommunicator
 * 通信基类
 */
BaseCommunicator::~BaseCommunicator() {}

BaseCommunicator::BaseCommunicator(const int port) : port(port) {
    this->debug_messages = DebugMessages({
        {"get_local_ip_failed", "获取本地 IP 失败: {}"},
        {"socket_create_failed", "Socket 创建失败"},
        {"socket_bind_failed", "Socket 绑定失败"},
        {"socket_listen_failed", "Socket 监听失败"},
        {"socket_accept_failed", "Socket 接受失败"},
        {"wait_connection", "等待连接..."},
        {"connect_success", "连接成功"},
        {"connect_failed", "连接失败"},
        {"send_message_failed", "发送消息失败"},
        {"recv_message_failed", "接收消息失败"},
        {"send_message_success", "发送消息成功"},
        {"recv_message_success", "接收消息成功"},
    });
}

int BaseCommunicator::createServerSocket() {
    struct sockaddr_in sockaddr;
    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(this->port); // 端口号

    char ip_local[32 + 1] = {0};
    if (!getLocalIp(ip_local)) {
        Debug::CoutError(debug_messages["get_local_ip_failed"], ip_local);
        exit(0);
    }
    inet_aton(ip_local, &sockaddr.sin_addr); // 将一个字符串IP地址转换为一个32位的网络序列IP地址

    this->server_socket_fd = socket(AF_INET, SOCK_STREAM, 0); // 创建套接字

    if (this->server_socket_fd < 0) {
        Debug::CoutError(debug_messages["socket_create_failed"]);
        exit(0);
    }

    if (bind(this->server_socket_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) { // 绑定套接字
        Debug::CoutError(debug_messages["socket_bind_failed"]);
        close(this->server_socket_fd);
        exit(0);
    }
    if (listen(this->server_socket_fd, 1) != 0) { // 监听套接字
        Debug::CoutError(debug_messages["socket_listen_failed"]);
        close(this->server_socket_fd);
        exit(0);
    }
    return this->server_socket_fd;
}

void BaseCommunicator::acceptConnection(std::function<void()> onConnect) {
    struct sockaddr_in *addr = (struct sockaddr_in *)malloc(sizeof(struct sockaddr_in));
    socklen_t addr_len = (socklen_t)sizeof(*addr);
    memset(addr, 0, sizeof(*addr));

    Debug::CoutDebug(debug_messages["wait_connection"]);
    this->fd = accept(this->server_socket_fd, (struct sockaddr *)addr, &addr_len);

    if (this->fd < 0) {
        Debug::CoutDebug(debug_messages["socket_accept_failed"]);
        close(this->server_socket_fd);
        free(addr);
        exit(0);
    } else {
        Debug::CoutSuccess(debug_messages["connect_success"]);
        if (onConnect != nullptr) {
            onConnect();
        }
    }
}

bool BaseCommunicator::sendData(const unsigned char *send_buffer, const int send_length) {
    unique_lock<mutex> lock(this->mutex_);
    if (write(this->fd, send_buffer, send_length) < 0) {
        Debug::CoutError(debug_messages["send_message_failed"]);
        return false;
    }
    return true;
}

int BaseCommunicator::recvData(unsigned char *recv_buffer, const int recv_length) {
    int len = -1;
    while ((len = recv(this->fd, recv_buffer, recv_length, 0)) < 0)
        ;
    return len;
}

bool BaseCommunicator::sendMessage(google::protobuf::Message &message) {
    ostringstream output_stream(ios::binary);
    // 写入消息长度
    int message_size = message.ByteSizeLong();
    output_stream.write(reinterpret_cast<const char *>(&message_size), sizeof(message_size));

    if (!message.SerializeToOstream(&output_stream)) {
        Debug::CoutError("序列化消息失败");
        return false;
    }
    // 获取序列化后的数据并发送到网络对端
    string serialized_data = output_stream.str();
    return sendData((unsigned char *)serialized_data.data(), serialized_data.size());
}

bool BaseCommunicator::recvMessage(etrs::proto::DataMessage &message) {
    unsigned char recv_buffer[1024];
    int len = recvData(recv_buffer, 1024);
    if (!message.ParseFromArray(recv_buffer, len)) {
        return false;
    }
    return true;
}

/**
 * PythonCommunicator
 * 用于与本地 Python 进程通信
 */
// PythonCommunicator::~PythonCommunicator() {}

// PythonCommunicator::PythonCommunicator(const int port) : BaseCommunicator(port) {
//     this->debug_messages.updateMessages(DebugMessages({{"wait_connection", "等待 Python 进程连接..."},
//                                                        {"connect_success", "Python 进程连接成功"},
//                                                        {"connect_failed", "Python 进程连接失败"}}));
// }

// int PythonCommunicator::sendMessageFromPointCloud(open3d::geometry::PointCloud point_cloud, const int interval) {
//     if (point_cloud.IsEmpty()) {
//         Debug::CoutError("Point cloud 为空");
//         return -1;
//     }
//     etrs::proto::DataMessage data_message;
//     data_message.set_type(etrs::proto::DataMessage::POINT_CLOUD);
//     etrs::proto::PointCloud *point_cloud_message = data_message.mutable_point_cloud();
//     const vector<Eigen::Vector3d> &points = point_cloud.points_;

//     int write_count = 0;
//     int point_size = points.size();
//     for (int i = 0; i < point_size; i++) {
//         etrs::proto::Point *p = point_cloud_message->add_points();
//         p->set_x(points[i][0]);
//         p->set_y(points[i][1]);
//         p->set_z(points[i][2]);

//         if ((i + 1) % interval == 0 || i == (point_size - 1)) {
//             // unique_lock<mutex> lock(PythonCommunicator_mutex);
//             sendMessage(data_message);
//             point_cloud_message->Clear();
//             write_count++;
//         }
//         Debug::CoutFlush("已发送：{}", write_count);
//     }
//     Debug::CoutSection("发送完毕", "一共发送了 {} 次\n 面片数量 {} ", write_count, point_size);
//     // sendExitMeshMessage();
//     return write_count;
// }

/**
 * HoloCommunicator
 * 用于与 HoloLens 2 客户端通信
 */
HoloCommunicator::~HoloCommunicator() {}

HoloCommunicator::HoloCommunicator(const int port) : BaseCommunicator(port) {
    this->debug_messages.updateMessages(DebugMessages({{"wait_connection", "等待 Holo 客户端连接..."},
                                                       {"connect_success", "Holo 客户端连接成功"},
                                                       {"connect_failed", "Holo 客户端连接失败"}}));
}

HoloCommunicator::HoloCommunicator(const int port, int server_socket_fd) : BaseCommunicator(port) {
    this->server_socket_fd = server_socket_fd;
}

// 发送Protobuf消息到网络对端
// bool HoloCommunicator::sendMessage(google::protobuf::Message &message) {
//     ostringstream output_stream(ios::binary);
//     int message_size = message.ByteSizeLong();
//     output_stream.write(reinterpret_cast<const char *>(&message_size), sizeof(message_size));

//     if (!message.SerializeToOstream(&output_stream)) {
//         Debug::CoutError("序列化消息失败");
//         return false;
//     }
//     // 获取序列化后的数据并发送到网络对端
//     string serialized_data = output_stream.str();
//     // FIXME: 待验证
//     return sendData((unsigned char *)serialized_data.data(), serialized_data.size());
//     // unique_lock<mutex> lock(this->mutex_);
//     // if (write(this->fd, serialized_data.data(), serialized_data.size()) < 0) {
//     //     Debug::CoutError("发送消息失败");
//     //     return false;
//     // }
//     // return true;

// }

// bool HoloCommunicator::recvMessage(etrs::proto::DataMessage &message) {
//     unsigned char recv_buffer[1024];
//     int len = recvData(recv_buffer, 1024);
//     if (!message.ParseFromArray(recv_buffer, len)) {
//         return false;
//     }
//     return true;
// }

// 发送exit_mesh
bool HoloCommunicator::sendExitMeshMessage() {
    etrs::proto::DataMessage message;
    message.set_type(etrs::proto::DataMessage::EXIT_MESH);
    return sendMessage(message);
}

int HoloCommunicator::sendMessageFromMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr,
                                          const int interval) {
    if (mesh_ptr == nullptr) {
        Debug::CoutError("Mesh 为空");
        return -1;
    }
    etrs::proto::DataMessage data_message;
    data_message.set_type(etrs::proto::DataMessage::MESH); // 设置消息类型
    etrs::proto::Mesh *point_cloud_message = data_message.mutable_mesh();
    const vector<Eigen::Vector3d> &vertices = mesh_ptr->vertices_;    // 顶点坐标
    const vector<Eigen::Vector3i> &triangles = mesh_ptr->triangles_;  // 顶点索引
    const vector<Eigen::Vector3d> &colors = mesh_ptr->vertex_colors_; // 顶点颜色
    int write_count = 0;
    int triangle_size = triangles.size();
    for (int i = 0; i < triangle_size; i++) {
        etrs::proto::Vertex *v1 = point_cloud_message->add_v1();
        int v1_index = triangles[i][0];
        v1->set_x(vertices[v1_index][0]);
        v1->set_y(vertices[v1_index][1]);
        v1->set_z(vertices[v1_index][2]);

        etrs::proto::Vertex *v2 = point_cloud_message->add_v2();
        int v2_index = triangles[i][1];
        v2->set_x(vertices[v2_index][0]);
        v2->set_y(vertices[v2_index][1]);
        v2->set_z(vertices[v2_index][2]);

        etrs::proto::Vertex *v3 = point_cloud_message->add_v3();
        int v3_index = triangles[i][2];
        v3->set_x(vertices[v3_index][0]);
        v3->set_y(vertices[v3_index][1]);
        v3->set_z(vertices[v3_index][2]);

        point_cloud_message->add_r((colors[v1_index][0] + colors[v2_index][0] + colors[v3_index][0]) / 3.0);
        point_cloud_message->add_g((colors[v1_index][1] + colors[v2_index][1] + colors[v3_index][1]) / 3.0);
        point_cloud_message->add_b((colors[v1_index][2] + colors[v2_index][2] + colors[v3_index][2]) / 3.0);

        if ((i + 1) % interval == 0 || i == (triangle_size - 1)) {
            // unique_lock<mutex> lock(HoloCommunicator_mutex);
            sendMessage(data_message);
            point_cloud_message->Clear();
            write_count++;
        }
        Debug::CoutFlush("已发送：{}", write_count);
    }
    Debug::CoutSection("发送完毕", "一共发送了 {} 次\n 面片数量 {} ", write_count, triangle_size);
    sendExitMeshMessage();
    return write_count;
}

int HoloCommunicator::sendMessageFromMesh(open3d::geometry::TriangleMesh mesh, const int interval) {
    if (mesh.IsEmpty()) {
        Debug::CoutError("Mesh 为空");
        return -1;
    }
    etrs::proto::DataMessage data_message;
    data_message.set_type(etrs::proto::DataMessage::MESH);
    etrs::proto::Mesh *point_cloud_message = data_message.mutable_mesh();
    const vector<Eigen::Vector3d> &vertices = mesh.vertices_;
    const vector<Eigen::Vector3i> &triangles = mesh.triangles_;
    const vector<Eigen::Vector3d> &colors = mesh.vertex_colors_;

    int write_count = 0;
    int triangle_size = triangles.size();
    for (int i = 0; i < triangle_size; i++) {
        etrs::proto::Vertex *v1 = point_cloud_message->add_v1();
        int v1_index = triangles[i][0];
        v1->set_x(vertices[v1_index][0]);
        v1->set_y(vertices[v1_index][1]);
        v1->set_z(vertices[v1_index][2]);

        etrs::proto::Vertex *v2 = point_cloud_message->add_v2();
        int v2_index = triangles[i][1];
        v2->set_x(vertices[v2_index][0]);
        v2->set_y(vertices[v2_index][1]);
        v2->set_z(vertices[v2_index][2]);

        etrs::proto::Vertex *v3 = point_cloud_message->add_v3();
        int v3_index = triangles[i][2];
        v3->set_x(vertices[v3_index][0]);
        v3->set_y(vertices[v3_index][1]);
        v3->set_z(vertices[v3_index][2]);

        point_cloud_message->add_r((colors[v1_index][0] + colors[v2_index][0] + colors[v3_index][0]) / 3.0);
        point_cloud_message->add_g((colors[v1_index][1] + colors[v2_index][1] + colors[v3_index][1]) / 3.0);
        point_cloud_message->add_b((colors[v1_index][2] + colors[v2_index][2] + colors[v3_index][2]) / 3.0);

        if ((i + 1) % interval == 0 || i == (triangle_size - 1)) {
            // unique_lock<mutex> lock(HoloCommunicator_mutex);
            sendMessage(data_message);
            point_cloud_message->Clear();
            write_count++;
        }
        Debug::CoutFlush("已发送：{}", write_count);
    }
    Debug::CoutSection("发送完毕", "一共发送了 {} 次\n 面片数量 {} ", write_count, triangle_size);
    sendExitMeshMessage();
    return write_count;
}

int HoloCommunicator::sendMessageFromDetectionResult(DetectionResultType detection_result) {
    etrs::proto::DataMessage data_message;
    data_message.set_type(etrs::proto::DataMessage::DETECTION_RESULT);
    etrs::proto::DetectionResult *detection_result_message = data_message.mutable_detection_result();
    for (auto item : detection_result) {
        etrs::proto::Obj *obj = detection_result_message->add_objs();
        obj->set_label(item.label);
        obj->set_score(item.score);
        etrs::proto::BoundingBox *bbox = obj->mutable_bbox();
        bbox->set_x(item.bbox.x);
        bbox->set_y(item.bbox.y);
        bbox->set_z(item.bbox.z);
        bbox->set_l(item.bbox.l);
        bbox->set_w(item.bbox.w);
        bbox->set_h(item.bbox.h);
        bbox->set_yaw(item.bbox.yaw);
    }
    return sendMessage(data_message);
}