#include "socket_comm.hpp"
#include "proto/obs.pb.h"
#include "ipc/DataDispatcher.hpp"
#include <thread>
#include <string.h>
namespace Comm {
/**
 * @brief SocketClient::Connect
 * @param port
 * @param ip
 * @return
 */
bool SocketClient::Connect(const int& port, const std::string& ip) {
  // 创建通信的套接字
  // int fd：
  //     这是一个整数类型的变量，名为fd。在套接字编程中，fd（通常称为文件描述符）是一个非负整数，用于唯一标识一个套接字。所有的套接字操作，如发送、接收、关闭等，都会使用这个fd。
  // **socket()**：
  //     这是一个系统调用，用于创建一个新的套接字。
  // AF_INET：
  //     这是socket()函数的第一个参数，表示要创建的套接字的地址族。AF_INET表示使用IPv4地址族。
  // SOCK_STREAM：
  //     这是socket()函数的第二个参数，表示要创建的套接字的类型。SOCK_STREAM表示创建一个流套接字，它提供了一个面向连接的、可靠的、基于字节流的传输层服务。这种套接字类型通常用于TCP协议。
  // 0：
  //     这是socket()函数的第三个参数，通常被称为协议。在这个例子中，它被设置为0，这意味着系统将自动选择适当的协议。对于SOCK_STREAM和AF_INET的组合，这通常是TCP协议。
  fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (fd_ == -1) {
    perror("socker");
    return 0;
  }
  // 绑定本地IP，port
  struct sockaddr_in address;
  address.sin_family = AF_INET;
  // 将点分十进制的 IP 地址（如 "192.168.1.1"）转换为网络字节序的二进制表示的函数
  inet_pton(AF_INET, ip.c_str(), &address.sin_addr.s_addr);
  address.sin_port = htons(port); // 端口号

  struct timeval timeout;  
  timeout.tv_sec = 2;  // 设置超时为2秒  
  timeout.tv_usec = 0;  
  socklen_t len = sizeof(timeout);  
  // 使用setsockopt函数为套接字fd_设置选项。这里设置的是发送超时选项SO_SNDTIMEO, 检查发送超时 
  // 用于后续的connect操作，若connect超时，则可以通过返回值和errno判断 
  // SOL_SOCKET表示选项应用于套接字级别。
  // len变量存储了timeout结构体的大小，这是setsockopt函数需要的。
  int ret = setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, len);
  assert(ret != -1);

  ret = connect(fd_, (struct sockaddr*)&address, sizeof(address));
  if (ret == -1) {
    if (errno == EINPROGRESS) {
      std::cout << "connecting timeout... " << "\n";
      return 0;
    }
    std::cout << "error occur when connecting to server " << "\n";
    return 0;
  }
  std::thread receive_thread(&SocketClient::receiveThread, this, fd_);
  receive_thread.detach();
  return 1;
}

/**
 * @brief SocketClient::Send
 * @param type
 * @param serialized_message
 */
void SocketClient::Send(const uint8_t& type, const std::string& serialized_message) {
  if (fd_ < 0) return;
  // 处理粘包的问题，加一个包头数据 
  // 获取消息信息
  MessageInfo info;
  info.message_type = type;
  info.message_len = serialized_message.size();
  // 发送长度信息
  send(fd_, &info, sizeof(info), 0);
  // 发送序列化数据主体
  send(fd_, serialized_message.data(), serialized_message.size(), 0);
}

/**
 * @brief SocketClient::receiveThread
 * @param fd
 */
void SocketClient::receiveThread(int fd) {
  while (1) {
    // std::cout << "receiveThread" << "\n"; 
    // 先接收序列化数据信息
    MessageInfo info;
    int len = recv(fd, &info, sizeof(info), 0);
    if (len <= 0) {
      continue;
    }
    std::string received_message;
    // 根据长度接收protobuf数据
    received_message.resize(info.message_len);
    ssize_t bytes_received = 0;
    while (bytes_received < info.message_len) {
        ssize_t bytes = recv(fd, received_message.data() + bytes_received, 
                                                  info.message_len - bytes_received, 0);
        if (bytes <= 0) {
            // 处理错误或连接关闭的情况
            break;
        }
        bytes_received += bytes;
    }
    ipc::DataDispatcher::GetInstance().Publish("ServerMsg", std::make_pair(info.message_type, received_message));
  }
  close(fd);
  return;
}
}

