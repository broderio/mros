#include "socket/socket.hpp"

Socket::Socket() {
    fd = -1;
}

Socket::Socket(int fd) {
    this->fd = fd;
}

Socket::~Socket() { }