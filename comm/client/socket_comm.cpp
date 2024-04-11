#include "socket_comm.hpp"
#include <thread>
#include <string.h>
namespace Comm {

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
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd == -1) {
    perror("socker");
    return -1;
  }
  // 绑定本地IP，port
  struct sockaddr_in address;
  address.sin_family = AF_INET;
  // 将点分十进制的 IP 地址（如 "192.168.1.1"）转换为网络字节序的二进制表示的函数
  inet_pton(AF_INET, ip.c_str(), &address.sin_addr.s_addr);
  address.sin_port = htons(port); // 端口号
  // 绑定套接字到地址和端口
  if (connect(fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
      perror("connect failed");
      return -1;
  }
  std::thread receive_thread(&SocketClient::receiveThread, this, fd);
  receive_thread.detach();
  return 1;
}

void SocketClient::receiveThread(int fd) {
  while (1) {
    char buff[1024];
    sprintf(buff, "hello, server");
    send(fd, buff, strlen(buff), 0);

    int len = recv(fd, buff, sizeof(buff), 0);
    if (len > 0) {
      std::cout << "server say: " << buff << "\n";
    } else if (len == 0) {
      std::cout << "断开连接" << "\n";
      break;
    } else {
      break;
    }
  }
  close(fd);
}

}
