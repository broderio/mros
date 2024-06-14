#include "socket/server.hpp"

TCPServer::TCPServer(const std::string &address, int port, int max_connections)
: max_connections(max_connections), num_connections(0) {

    // AF_INET specifies that we are using the IPv4 protocol
    server.sin_family = AF_INET;
    
    // htons() converts the port number from host byte order to network byte order
    server.sin_port = htons(port);

    // inet_aton() converts the address from a string to a binary representation
    if (inet_aton(address.c_str(), &server.sin_addr) == 0) {
        std::cerr << "TCPServer error: invalid address" << std::endl;
        return;
    }

    // Create a socket 
    // AF_INET specifies that we are using the IPv4 protocol
    // SOCK_STREAM specifies that we are using the TCP protocol
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        std::cerr << "TCPServer error: failed to create socket" << std::endl;
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

    // Bind the socket to the address and port
    if (::bind(fd, (struct sockaddr *)&server, sizeof(server)) < 0) {
        std::cerr << "TCPServer error: bind failed" << std::endl;
        ::close(fd);
        return;
    }

    // Listen for incoming connections
    if (::listen(fd, max_connections) < 0) {
        std::cerr << "TCPServer error: listen failed" << std::endl;
        ::close(fd);
        return;
    }
}

TCPServer::~TCPServer() {
    close();
}

int TCPServer::accept(TCPConnection& connection) {

    // If maximum number of connections has been reached
    if (num_connections >= max_connections) {
        std::cerr << "TCPServer error: maximum number of connections reached" << std::endl;
        return -1;
    }

    // Client information
    struct sockaddr_in client;
    socklen_t client_len = sizeof(client);

    // Accept a connection
    int client_fd = ::accept(fd, (struct sockaddr *)&client, &client_len);

    // If the accept call fails
    if (client_fd < 0) {

        // If the error is EWOULDBLOCK or EAGAIN, there are no pending connections
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            return 1;
        
        }
        // Otherwise, there was an error 
        else {
            std::cerr << "TCPServer error: accept failed" << std::endl;
            return -1;
        }
    }

    // Make the socket non-blocking
    int flags = fcntl(client_fd, F_GETFL, 0);
    if (flags < 0) {
        std::cerr << "TCPServer error: failed to get flags" << std::endl;
        ::close(client_fd);
        return -1;
    }
    flags |= O_NONBLOCK;
    if (fcntl(client_fd, F_SETFL, flags) < 0) {
        std::cerr << "TCPServer error: failed to set flags" << std::endl;
        ::close(client_fd);
        return -1;
    }

    connection.fd = client_fd;
    connection.isOpen = true;
    num_connections++;
    return 0;
}

void TCPServer::close() {
    ::close(fd);
}

TCPConnection::TCPConnection() {
    fd = -1;
    isOpen = false;
}

TCPConnection::~TCPConnection() {
    close();
}

int TCPConnection::send(const std::string& message) {
    // If the connection is closed (i.e. this object was not created by a server)
    if (!isOpen) {
        std::cerr << "TCPConnection error: connection is closed" << std::endl;
        return -1;
    }

    // Send the message
    if (::send(fd, message.c_str(), message.size(), 0) < 0) {
        std::cerr << "TCPConnection error: send failed" << std::endl;
        return -1;
    }
    return 0;
}

int TCPConnection::receive(std::string& message, size_t bytes) {
    // If the connection is closed (i.e. this object was not created by a server)
    if (!isOpen) {
        std::cerr << "TCPConnection error: connection is closed" << std::endl;
        message = "";
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

void TCPConnection::close() {
    ::close(fd);
    isOpen = false;
}