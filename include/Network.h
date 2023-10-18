#ifndef _NETWORK_H_
#define _NETWORK_H_

#include "DataMessage.pb.h"
#include <Utility.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <iostream>
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

    namespace proto {
        bool send_message(int fd, google::protobuf::Message &message);
        bool send_exit_mesh_message(int fd);
    } // namespace proto

    // class Client {
    // private:
    //     int port;
    //     int fd;
    //     mutex mutex_;

    // public:
    //     int server_socket_fd;

    // public:
    //     // 反馈函数
    //     explicit Client(const int port, std::function<void()> onConnect = nullptr);
    //     explicit Client(const int port, int server_socket_fd, std::function<void()> onConnect = nullptr);
    //     bool sendMessage(google::protobuf::Message &message);
    //     int recvData(unsigned char *recv_buffer, const int recv_length);
    //     bool recvMessage(etrs::proto::DataMessage &message);
    //     bool sendExitMeshMessage();
    //     int sendMessageFromMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr, const int interval);
    //     int sendMessageFromMesh(open3d::geometry::TriangleMesh mesh, const int interval);
    // };

    // class LocalProcessCommunicator : public Client {
    // private:
    //     int port;

    // public:
    //     explicit LocalProcessCommunicator(const int port, std::function<void()> onConnect = nullptr);
    // };

    class LocalProcessCommunicator {
    protected:
        int port;
        int fd = -1;
        mutex mutex_;
        
    public: 
        int server_socket_fd;

    public:
        explicit LocalProcessCommunicator(const int port);
        virtual int createServerSocket();
        virtual void acceptConnection(std::function<void()> onConnect = nullptr);
        bool sendData(const unsigned char *send_buffer, const int send_length);
        int recvData(unsigned char *recv_buffer, const int recv_length);

    private:
        // bool hasCreateSocket(); //检查是否未创建socket
    };

    class Client : public LocalProcessCommunicator {
    private:

    public:
        explicit Client(const int port);
        explicit Client(const int port, int server_socket_fd);
        int createServerSocket() override;
        void acceptConnection(std::function<void()> onConnect = nullptr) override;
        bool sendMessage(google::protobuf::Message &message);
        bool recvMessage(etrs::proto::DataMessage &message);
        bool sendExitMeshMessage();
        int sendMessageFromMesh(std::shared_ptr<open3d::geometry::TriangleMesh> mesh_ptr, const int interval);
        int sendMessageFromMesh(open3d::geometry::TriangleMesh mesh, const int interval);
    };

} // namespace etrs::net

#endif // _NETWORK_H_
