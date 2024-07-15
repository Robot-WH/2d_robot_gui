#pragma once
#include "common.hpp"
namespace Comm {
class SocketClient {
public:
  bool Connect(const int& port, const std::string& ip);
  void TcpSend(const uint8_t& type, const std::string& serialized_message);
  void UdpSend(const std::string& serialized_message);
protected:
  void receiveThread(int fd);
private:
  struct sockaddr_in address_;
  int fd_ = -1;
  int udp_fd_ = -1;
};
}
