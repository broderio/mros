#include "socket/client.hpp"

ClientSocket::ClientSocket(int domain, const std::string &address, int port) {
    server.sin_family = domain;
    server.sin_port = htons(port);
    if (inet_pton(domain, address.c_str(), &server.sin_addr) <= 0) {
        throw std::runtime_error("Invalid address");
    }

    fd = socket(domain, SOCK_STREAM, 0);
}

ClientSocket::~ClientSocket() {
    close();
}

void ClientSocket::connect() {
    if (::connect(fd, (struct sockaddr *)&server, sizeof(server)) < 0) {
        throw std::runtime_error("Connection failed");
    }
}

void ClientSocket::send(const std::string& message) {
    ::send(fd, message.c_str(), message.size(), 0);
}

std::string ClientSocket::receive() {
    char buffer[1024] = {0};
    int valread = read(fd, buffer, 1024);
    return std::string(buffer, valread);
}

void ClientSocket::close() {
    ::close(fd);
}