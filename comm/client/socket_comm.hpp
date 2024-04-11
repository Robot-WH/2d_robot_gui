#pragma once
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <string>
namespace Comm {
class SocketClient {
public:
  bool Connect(const int& port, const std::string& ip);
protected:
  void receiveThread(int fd);
};
}
