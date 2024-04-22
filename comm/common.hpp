#pragma once
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <string>

struct MessageInfo {
  uint8_t message_type;
  int32_t message_len;
};
