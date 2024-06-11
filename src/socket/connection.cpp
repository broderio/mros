#include "socket/connection.hpp"

ConnectionSocket::ConnectionSocket(int fd) : Socket(fd) { }

ConnectionSocket::~ConnectionSocket() { }

void ConnectionSocket::send(const std::string& message) {
    ::send(fd, message.c_str(), message.size(), 0);
}

std::string ConnectionSocket::receive() {
    char buffer[1024] = {0};
    int valread = read(fd, buffer, 1024);
    return std::string(buffer, valread);
}

void ConnectionSocket::close() {
    ::close(fd);
}