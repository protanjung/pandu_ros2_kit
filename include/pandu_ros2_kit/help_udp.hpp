/**
 * @file help_udp.hpp
 * @author Pandu Surya Tantra (pandustantra@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef HELP_UDP_HPP_
#define HELP_UDP_HPP_

#include "arpa/inet.h"
#include "pandu_ros2_kit/help_logger.hpp"
#include "rclcpp/rclcpp.hpp"

class HelpUDP {
  private:
    bool _is_initialized = false;
    bool _is_server = false;
    bool _is_client = false;
    bool _is_client_address_set = false;

    HelpLogger _logger;

    int _socket_fd;
    struct sockaddr_in _socket_server_address;
    struct sockaddr_in _socket_client_address;
    socklen_t _socket_server_address_len;
    socklen_t _socket_client_address_len;
    uint8_t _socket_tx_buffer[65535];
    uint8_t _socket_rx_buffer[65535];

  public:
    HelpUDP();

    bool is_initialized();

    bool init_as_server(int port);
    bool init_as_client(std::string ip, int port, bool same_bind_port_as_server = false);

    size_t recv(void* buf, size_t len);
    size_t send(void* buf, size_t len);
};

#endif // HELP_UDP_HPP_