#pragma once
#include "common.hpp"
namespace Comm {
class SocketClient {
public:
  bool Connect(const int& port, const std::string& ip);
  void Send(const uint8_t& type, const std::string& serialized_message);
protected:
  void receiveThread(int fd);
private:
    int fd_ = -1;
};
}
