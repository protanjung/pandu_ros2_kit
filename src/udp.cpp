
#include "pandu_ros2_kit/udp.hpp"

UDP::UDP() {}

//======================================

bool UDP::init_as_server(int port) {
  if (_is_initialized) { return true; }

  /* The code is creating a socket for communication. */
  if ((_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    std::cerr << "Socket creation failed" << std::endl;
    return false;
  }

  /* The code is filling the server information. */
  _socket_server_address.sin_family = AF_INET;
  _socket_server_address.sin_addr.s_addr = htonl(INADDR_ANY);
  _socket_server_address.sin_port = htons(port);
  _socket_server_address_len = sizeof(_socket_server_address);

  /* The code is binding the socket with the server address. */
  if (bind(_socket_fd, (const struct sockaddr *)&_socket_server_address, sizeof(_socket_server_address)) < 0) {
    std::cerr << "Bind failed" << std::endl;
    return false;
  }

  _is_initialized = _is_server = true;
  return true;
}

bool UDP::init_as_client(std::string ip, int port) {
  if (_is_initialized) { return true; }

  /* The code is creating a socket for communication. */
  if ((_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    std::cerr << "Socket creation failed" << std::endl;
    return false;
  }

  /* The code is filling the server information. */
  _socket_server_address.sin_family = AF_INET;
  _socket_server_address.sin_addr.s_addr = inet_addr(ip.c_str());
  _socket_server_address.sin_port = htons(port);
  _socket_server_address_len = sizeof(_socket_server_address);

  _is_initialized = _is_client = true;
  return true;
}

bool UDP::is_initialized() { return _is_initialized; }

//======================================

ssize_t UDP::recv(void *buf, size_t len) {
  ssize_t ret;

  ret = recvfrom(
      _socket_fd,
      buf,
      len,
      MSG_DONTWAIT,
      (struct sockaddr *)&_socket_client_address,
      (socklen_t *)&_socket_client_address_len);

  if (ret > 0) { _is_client_address_set = true; }

  return ret;
}

ssize_t UDP::send(void *buf, size_t len) {
  ssize_t ret;

  if (_is_client) {
    ret = sendto(
        _socket_fd,
        buf,
        len,
        MSG_DONTWAIT,
        (const struct sockaddr *)&_socket_server_address,
        (socklen_t)_socket_server_address_len);
  } else if (_is_server && _is_client_address_set) {
    ret = sendto(
        _socket_fd,
        buf,
        len,
        MSG_DONTWAIT,
        (const struct sockaddr *)&_socket_client_address,
        (socklen_t)_socket_client_address_len);
  } else {
    ret = -1;
  }

  return ret;
}