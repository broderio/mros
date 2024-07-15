#include "socket/tcp/client.hpp"

TCPClient::TCPClient()
: nonblocking(true), connected(false), opened(false) {}

TCPClient::TCPClient(const URI &uri, bool nonblocking)
: nonblocking(nonblocking), connected(false), opened(false) {
    open(uri, nonblocking);
}

TCPClient::~TCPClient() {
    close();
}

int TCPClient::open(const URI &uri, bool nonblocking) {
    if (opened) {
        //std::cerr << "TCPClient error: already open" << std::endl;
        return -1;
    }

    // AF_INET specifies that we are using the IPv4 protocol
    server.sin_family = AF_INET;

    // htons() converts the port number from host byte order to network byte order
    server.sin_port = htons(uri.port);

    // inet_aton() converts the address from a string to a binary representation
    if (inet_aton(uri.ip.c_str(), &server.sin_addr) == 0) {
        //std::cerr << "TCPClient error: invalid address" << std::endl;
        return -1;
    }

    // Create a socket
    // AF_INET specifies that we are using the IPv4 protocol
    // SOCK_STREAM specifies that we are using the TCP protocol
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        //std::cerr << "TCPClient error: failed to create socket" << std::endl;
        return -1;
    }

    // Make the socket non-blocking
    if (nonblocking) {
        int flags = fcntl(fd, F_GETFL, 0);
        if (flags < 0) {
            //std::cerr << "TCPClient error: failed to get flags" << std::endl;
            ::close(fd);
            return -1;
        }
        flags |= O_NONBLOCK;
        if (fcntl(fd, F_SETFL, flags) < 0) {
            //std::cerr << "TCPClient error: failed to set flags" << std::endl;
            ::close(fd);
            return -1;
        }
    }
    this->nonblocking = nonblocking;

    opened = true;
    return 0;
}

// TODO: Add comments for this function
int TCPClient::connect(int timeout) {
    if (connected) {
        //std::cerr << "TCPClient error: already connected" << std::endl;
        return -1;
    }

    if (::connect(fd, (struct sockaddr *)&server, sizeof(server)) < 0) {
        if (errno == EINPROGRESS) {
            return 1;
        }
        if (errno == EISCONN) {
            connected = true;
            return 2;
        }
        //perror("TCPClient connect");
        return -1;
    }
    connected = true;
    return 0;
}

int TCPClient::send(const std::string& message) {
    // If a connection to the server has not been made yet
    if (!connected) {
        //std::cerr << "TCPClient error: not connected" << std::endl;
        return -1;
    }

    // Send the message
    if (::send(fd, message.c_str(), message.size(), 0) < 0) {
        close();
        //std::cerr << "TCPClient error: send failed" << std::endl;
        return -1;
    }
    return 0;
}

int TCPClient::receive(std::string& message, size_t bytes) {
    // If a connection to the server has not been made yet
    if (!connected) {
        //std::cerr << "TCPClient error: not connected" << std::endl;
        return -1;
    }

     // Receive the message, reading up to the specified number of bytes
    std::string buffer(bytes, 0);
    int bytes_received = ::recv(fd, &buffer[0], bytes, 0);
    if (bytes_received < 0) {
        message = "";
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // No data available to read
            return 0;
        } else {
            //perror("TCPClient error: receive failed");
            return -1;
        }
    }

    message = buffer.substr(0, bytes_received);
    return 0;
}

void TCPClient::close() {
    ::close(fd);
}

bool TCPClient::isOpen() const {
    return opened;
}

bool TCPClient::isConnected() const {
    return connected;
}

bool TCPClient::isNonblocking() const {
    return nonblocking;
}

URI TCPClient::getServerURI() {
    return URI(inet_ntoa(server.sin_addr), ntohs(server.sin_port));
}