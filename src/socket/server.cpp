#include "socket/server.hpp"

ServerSocket::ServerSocket(int domain, const std::string &address, int port) {
    server.sin_family = domain;
    server.sin_port = htons(port);
    if (inet_aton(address.c_str(), &server.sin_addr) == 0) {
        throw std::runtime_error("Invalid address");
    }

    fd = socket(domain, SOCK_STREAM, 0);
    if (fd < 0) {
        throw std::runtime_error("Socket creation failed");
    }
    int flags = fcntl(fd, F_GETFL, 0);  
    if (flags < 0) {
        throw std::runtime_error("Fcntl failed");
    }
    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
        throw std::runtime_error("Failed to set socket as nonblocking");
    }
    int opt = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        throw std::runtime_error("Failed to set socket as reusable");
    }

    if (bind(fd, (struct sockaddr *)&server, sizeof(server)) < 0) {
        throw std::runtime_error("Bind failed");
    }

    if (listen(fd, 5) < 0) {
        throw std::runtime_error("Listen failed");
    }
}

ServerSocket::~ServerSocket() {
    close();
}

ConnectionSocket ServerSocket::accept() {
    struct sockaddr_in client;
    socklen_t client_len = sizeof(client);
    int client_fd = ::accept(fd, (struct sockaddr *)&client, &client_len);
    if (client_fd < 0) {
        throw std::runtime_error("Accept failed");
    }
    return ConnectionSocket(client_fd);
}

void ServerSocket::close() {
    ::close(fd);
}