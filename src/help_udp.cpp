/**
 * @file help_udp.cpp
 * @author Pandu Surya Tantra (pandustantra@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-28
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "pandu_ros2_kit/help_udp.hpp"

HelpUDP::HelpUDP() {}

bool
HelpUDP::is_initialized() {
    return _is_initialized;
}

bool
HelpUDP::init_as_server(int port) {
    if (_is_initialized) {
        return true;
    }

    if (!_logger.init()) {
        return false;
    }

    if ((_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        _logger.error("Socket creation failed");
        return false;
    }

    _socket_server_address.sin_family = AF_INET;
    _socket_server_address.sin_addr.s_addr = htonl(INADDR_ANY);
    _socket_server_address.sin_port = htons(port);
    _socket_server_address_len = sizeof(_socket_server_address);

    if (bind(_socket_fd, (struct sockaddr*)&_socket_server_address, _socket_server_address_len) < 0) {
        _logger.error("Socket bind failed");
        return false;
    }

    _is_initialized = _is_server = true;
    return true;
}

bool
HelpUDP::init_as_client(std::string ip, int port, bool same_bind_port_as_server) {
    if (_is_initialized) {
        return true;
    }

    if (!_logger.init()) {
        return false;
    }

    if ((_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        _logger.error("Socket creation failed");
        return false;
    }

    _socket_server_address.sin_family = AF_INET;
    _socket_server_address.sin_addr.s_addr = inet_addr(ip.c_str());
    _socket_server_address.sin_port = htons(port);
    _socket_server_address_len = sizeof(_socket_server_address);

    if (same_bind_port_as_server) {
        _socket_client_address.sin_family = AF_INET;
        _socket_client_address.sin_addr.s_addr = htonl(INADDR_ANY);
        _socket_client_address.sin_port = htons(port);
        _socket_client_address_len = sizeof(_socket_client_address);

        if (bind(_socket_fd, (struct sockaddr*)&_socket_client_address, _socket_client_address_len) < 0) {
            _logger.error("Socket bind failed");
            return false;
        }
    }

    _is_initialized = _is_client = true;
    return true;
}

size_t
HelpUDP::recv(void* buf, size_t len) {
    if (!_is_initialized) {
        return -1;
    }

    ssize_t ret;

    if (_is_client) {
        ret = recvfrom(_socket_fd, buf, len, MSG_DONTWAIT, NULL, NULL);
    } else if (_is_server) {
        ret = recvfrom(_socket_fd, buf, len, MSG_DONTWAIT, (struct sockaddr*)&_socket_client_address,
                       &_socket_client_address_len);
    }

    if (_is_server && !_is_client_address_set && ret >= 0) {
        _is_client_address_set = true;
    }

    return ret;
}

size_t
HelpUDP::send(void* buf, size_t len) {
    if (!_is_initialized) {
        return -1;
    }

    if (_is_client) {
        return sendto(_socket_fd, buf, len, MSG_DONTWAIT, (struct sockaddr*)&_socket_server_address,
                      _socket_server_address_len);
    } else if (_is_server && _is_client_address_set) {
        return sendto(_socket_fd, buf, len, MSG_DONTWAIT, (struct sockaddr*)&_socket_client_address,
                      _socket_client_address_len);
    } else {
        return -1;
    }
}