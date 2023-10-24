#ifndef UDP_HPP_
#define UDP_HPP_

#include "arpa/inet.h"
#include "rclcpp/rclcpp.hpp"

class UDP {
 private:
  int _socket_fd;
  struct sockaddr_in _socket_server_address;
  struct sockaddr_in _socket_client_address;
  socklen_t _socket_server_address_len;
  socklen_t _socket_client_address_len;
  uint8_t _socket_tx_buffer[2048];
  uint8_t _socket_rx_buffer[2048];

  bool _is_initialized = false;
  bool _is_server = false;
  bool _is_client = false;

  bool _is_client_address_set = false;

 public:
  UDP();

  bool init_as_server(int port);
  bool init_as_client(std::string ip, int port);
  bool is_initialized();

  ssize_t recv(void *buf, size_t len);
  ssize_t send(void *buf, size_t len);
};

#endif  // UDP_HPP_