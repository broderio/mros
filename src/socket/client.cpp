#include "socket/client.hpp"

TCPClient::TCPClient(const std::string &address, int port) {
    // AF_INET specifies that we are using the IPv4 protocol
    server.sin_family = AF_INET;

    // htons() converts the port number from host byte order to network byte order
    server.sin_port = htons(port);

    // inet_aton() converts the address from a string to a binary representation
    if (inet_aton(address.c_str(), &server.sin_addr) == 0) {
        throw std::runtime_error("Invalid address");
    }

    // Create a socket
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        std::cerr << "TCPClient error: socket creation failed" << std::endl;
        return;
    }

    // Make the socket non-blocking
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        std::cerr << "TCPServer error: failed to get flags" << std::endl;
        ::close(fd);
        return;
    }
    flags |= O_NONBLOCK;
    if (fcntl(fd, F_SETFL, flags) < 0) {
        std::cerr << "TCPServer error: failed to set flags" << std::endl;
        ::close(fd);
        return;
    }

    isConnected = false;
}

TCPClient::~TCPClient() {
    close();
}

int TCPClient::connect() {
    if (::connect(fd, (struct sockaddr *)&server, sizeof(server)) < 0) {
        if (errno == EINPROGRESS) {
            // Connection in progress
            fd_set writefds;
            FD_ZERO(&writefds);
            FD_SET(fd, &writefds);
            struct timeval tv;
            tv.tv_sec = 5; // Timeout after 5 seconds
            tv.tv_usec = 0;
            if (select(fd + 1, NULL, &writefds, NULL, &tv) > 0) {
                int so_error;
                socklen_t len = sizeof so_error;
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_error, &len);
                if (so_error == 0) {
                    isConnected = true;
                    return 0;
                }
            }
        }
        close();
        std::cerr << "TCPClient error: connection failed" << std::endl;
        return -1;
    }
    isConnected = true;
    return 0;
}

int TCPClient::send(const std::string& message) {
    // If a connection to the server has not been made yet
    if (!isConnected) {
        std::cerr << "TCPClient error: not connected" << std::endl;
        return -1;
    }

    // Send the message
    if (::send(fd, message.c_str(), message.size(), 0) < 0) {
        close();
        std::cerr << "TCPClient error: send failed" << std::endl;
        return -1;
    }
    return 0;
}

int TCPClient::receive(std::string& message, size_t bytes) {
    // If a connection to the server has not been made yet
    if (!isConnected) {
        std::cerr << "TCPClient error: not connected" << std::endl;
        return -1;
    }

     // Receive the message, reading up to the specified number of bytes
    std::string buffer(bytes, 0);
    int bytes_received = ::recv(fd, &buffer[0], bytes, 0);
    if (bytes_received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available to read
            message = "";
            return 0;
        } else {
            perror("TCPClient error: receive failed");
            message = "";
            return -1;
        }
    }

    message = buffer.substr(0, bytes_received);
    return 0;
}

void TCPClient::close() {
    ::close(fd);
}