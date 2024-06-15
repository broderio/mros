#include "socket/udp/client.hpp"

UDPClient::UDPClient(const std::string &address, int port)
: isConnected(false) {
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);

    // inet_aton() converts the address from a string to a binary representation
    if (inet_aton(address.c_str(), &serverAddr.sin_addr) == 0) {
        std::cerr << "UDPClient error: invalid server address" << std::endl;
        return;
    }

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "UDPClient error: socket creation failed" << std::endl;
        return;
    }
}

UDPClient::~UDPClient() {
    close();
}

int UDPClient::connect() {
    if (::connect(fd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        std::cerr << "UDPClient error: connection failed" << std::endl;
        return -1;
    }
    isConnected = true;
    return 0;
}

int UDPClient::sendTo(const std::string& message) {
    if (!isConnected) {
        std::cerr << "UDPClient error: not connected" << std::endl;
        return -1;
    }
    
    return sendto(fd, message.c_str(), message.size(), 0, (struct sockaddr *)NULL, sizeof(serverAddr));
}

int UDPClient::receiveFrom(std::string& message, size_t bytes) {
    if (!isConnected) {
        std::cerr << "UDPClient error: not connected" << std::endl;
        return -1;
    }

    std::string buffer(bytes, 0);
    socklen_t serverAddrLen = sizeof(serverAddr);
    int bytes_received = recvfrom(fd, &buffer[0], bytes, 0, (struct sockaddr *)&serverAddr, &serverAddrLen);
    if (bytes_received < 0) {
        std::cerr << "UDPClient error: receive failed" << std::endl;
        message = "";
        return -1;
    }

    message = buffer.substr(0, bytes_received);
    return 0;
}

void UDPClient::close() {
    ::close(fd);
    isConnected = false;
}