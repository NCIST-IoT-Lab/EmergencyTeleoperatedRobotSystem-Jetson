
#ifndef _NETWORK_H_
#define _NETWORK_H_

#include "Utility.h"
#include "DataMessage.pb.h"

#include <iostream>
#include <arpa/inet.h>
#include <fcntl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <netinet/tcp.h> //TCP_NODELAY
#include <open3d/Open3D.h>
#include <sstream>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace etrs::utility;

namespace etrs::net {
    // bool getLocalIp(char *ip);

    // int creatServerSocket(int port);

    //TODO: 代码感觉还需要重构优化，可以分为两个基类，一个是基于TCP的通信，一个是基于Protobuf的通信。派生类多继承于这两个类
    class BaseCommunicator {    //基于 TCP 的通信
    protected:
        int port;
        int fd = -1;
        mutex mutex_;
        
    public: 
        DebugMessages debug_messages;
        int server_socket_fd;

    public:
        explicit BaseCommunicator(const int port);
        ~BaseCommunicator();
        int createServerSocket();
        void acceptConnection(std::function<void()> onConnect = nullptr);
        bool sendData(const unsigned char *send_buffer, const int send_length);
        int recvData(unsigned char *recv_buffer, const int recv_length);

        // TODO: 是否能剥离出来
        bool sendMessage(google::protobuf::Message &message);
        bool recvMessage(etrs::proto::DataMessage &message);

    private:
        // bool hasCreateSocket(); //检查是否未创建socket
    };
    

    class PythonCommunicator : public BaseCommunicator {
    private:

    public:
        explicit PythonCommunicator(const int port);
        int sendMessageFromMesh(open3d::geometry::TriangleMesh mesh, const int interval);
        // int sendMessageFromMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr, const int interval);
    
    private:
    };


    class HoloCommunicator : public BaseCommunicator {
    private:

    public:
        explicit HoloCommunicator(const int port);
        explicit HoloCommunicator(const int port, int server_socket_fd);
        // int createServerSocket() override;
        // void acceptConnection(std::function<void()> onConnect = nullptr) override;
        // bool sendMessage(google::protobuf::Message &message);
        // bool recvMessage(etrs::proto::DataMessage &message);
        bool sendExitMeshMessage();
        // TODO: 两个函数的函数体可以合并
        int sendMessageFromMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr, const int interval);
        int sendMessageFromMesh(open3d::geometry::TriangleMesh mesh, const int interval);
    };

} // namespace etrs::net

#endif // _NETWORK_H_
