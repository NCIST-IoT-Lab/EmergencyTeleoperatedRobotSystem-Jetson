#include "Bot.h"
#include "DataMessage.pb.h"
#include "Network.h"

#include <iostream>
#include <string>

#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <fcntl.h>
#include <iostream>
#include <net/if.h>
#include <netinet/in.h>
#include <netinet/tcp.h> //TCP_NODELAY
#include <sstream>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <open3d/Open3D.h>

using namespace std;
using namespace open3d;
using namespace etrs::utility;

bool getLocalIp_(char *ip) {
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

int main(int argc, char **argv) { // TODO: 可以传参，传入配置文件路径

    // 创建服务器等待连接
    // etrs::net::Client client(5001);
    int fd = -1;

    thread receive_client_thread([&]() {
        etrs::bot::BotArm bot_arm("/dev/ttyACM0");

        bot_arm.reset();
        Debug::CoutSuccess("机械臂复位成功");

        char client_buffer[1024];
        while (true) {
            while (fd < 0)
                ;

            etrs::proto::DataMessage data_message;
            unsigned char recv_buffer[1024];
            int len = 0;
            // while (len = recv(fd, recv_buffer, 1024, 0) <= 0) {
            //     continue;
            // }
            len = recv(fd, recv_buffer, 1024, 0);
            cout << "len: " << len << endl;;
           
            if (len <= 0 ||  !data_message.ParseFromArray(recv_buffer, len)) {
                continue;
            }
            switch ((int)data_message.type()) {
                case (int)etrs::proto::DataMessage::BOT_MOTOR: {
                    Debug::CoutSuccess("收到重建请求");
                    break;
                }
                case (int)etrs::proto::DataMessage::BOT_CAR: {
                    Debug::CoutSuccess("收到机器人数据");
                    int seq_length = data_message.bot_car().move_sequence_size();
                    int sequence_flag = data_message.bot_car().move_sequence(0);
                    Debug::CoutDebug("序列长度: {}", seq_length);
                    break;
                }
                case (int)etrs::proto::DataMessage::BOT_ARM: {
                    Debug::CoutSuccess("收到机械臂数据");
                    int length = data_message.bot_arm().data_buffer().length();
                    bot_arm.execute(data_message.bot_arm().data_buffer().data(), length);
                    bot_arm.sendCommand(etrs::bot::BotArm::CommandSet::READ_ANGLE);
                    // for (int i = 0; i < length; i++) {
                    //     cout << "jxb:" << data_message.bot_arm().data_buffer().data() << endl;
                    // }
                    break;
                }
                case (int)etrs::proto::DataMessage::BOT_GRIPPER: {
                    int status = data_message.bot_gripper().status();
                    Debug::CoutSuccess("收到机械臂夹爪数据: {}", status);
                    if (status == 1) {
                        bot_arm.openGripper(0x32);
                    } else if (status == 0) {
                        bot_arm.closeGripper(0x32);
                    }
                    break;
                }
                case (int)etrs::proto::DataMessage::KINECT_MODE: {
                    Debug::CoutSuccess("收到Kinect模式切换请求");
                    break;
                }
                case (int)etrs::proto::DataMessage::OTHER:
                default:
                    Debug::CoutError("未知的客户端数据类型");
                    // fd = -1;
                    break;
            }
        }
    });

    int server_socket_fd = -1;
    struct sockaddr_in *addr = (struct sockaddr_in *)malloc(sizeof(struct sockaddr_in));
    socklen_t addr_len = (socklen_t)sizeof(*addr);
    memset(addr, 0, sizeof(*addr));

    struct sockaddr_in sockaddr;
    memset(&sockaddr, 0, sizeof(sockaddr));
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(5001); // 端口号

    char ip_local[32 + 1] = {0};
    if (!getLocalIp_(ip_local)) {
        Debug::CoutError("连接IP失败: {}", ip_local);
        exit(0);
    }
    inet_aton(ip_local, &sockaddr.sin_addr); // 将一个字符串IP地址转换为一个32位的网络序列IP地址

    server_socket_fd = socket(AF_INET, SOCK_STREAM, 0); // 创建套接字

    if (server_socket_fd < 0) {
        Debug::CoutError("Socket 创建失败");
        exit(0);
    }

    if (bind(server_socket_fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0) { // 绑定套接字
        Debug::CoutError("Socket 绑定失败");
        close(server_socket_fd);
        exit(0);
    }
    if (listen(server_socket_fd, 1) != 0) { // 监听套接字
        Debug::CoutError("Socket 监听失败");
        close(server_socket_fd);
        exit(0);
    }

    while (true) {
        Debug::CoutDebug("等待客户端连接....");
        fd = accept(server_socket_fd, (struct sockaddr *)addr, &addr_len);

        if (fd < 0) {
            Debug::CoutError("Socket 接收失败");
            // close(server_socket_fd);
            // free(addr);
        } else {
            Debug::CoutSuccess("客户端连接成功，客户端IP: {}:{}", inet_ntoa(addr->sin_addr), ntohs(addr->sin_port));
        };
    }
    return 1;
}
