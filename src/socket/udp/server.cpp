#include "socket/udp/server.hpp"

UDPServer::UDPServer(int port) 
: isBound(false) {
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        std::cerr << "UDPServer error: failed to create socket" << std::endl;
        return;
    }
}
    
UDPServer::~UDPServer() {
    close();
}

int UDPServer::bind() {
    if (::bind(fd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("UDPServer: bind failed");
        ::close(fd);
        return -1;
    }
    isBound = true;
    return 0;
}

int UDPServer::sendTo(const std::string& message, const URI &clientURI) {
    if (!isBound) {
        std::cerr << "UDPServer error: not bound" << std::endl;
        return -1;
    }

    if (clientURIs.find(clientURI) == clientURIs.end()) {
        std::cerr << "UDPServer error: client IP not found" << std::endl;
        return -1;
    }

    sockaddr_in clientAddr;
    clientAddr.sin_family = AF_INET;
    clientAddr.sin_port = htons(clientURI.port);
    if (inet_aton(clientURI.ip.c_str(), &clientAddr.sin_addr) == 0) {
        std::cerr << "UDPClient error: invalid server address" << std::endl;
        return -1;
    }

    return ::sendto(fd, message.c_str(), message.size(), 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr));
}

int UDPServer::receiveFrom(std::string& message, size_t bytes, URI &clientURI) {
    if (!isBound) {
        std::cerr << "UDPServer error: not bound" << std::endl;
        return -1;
    }

    sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);

    std::string buffer(bytes, 0);
    int bytes_received = recvfrom(fd, &buffer[0], bytes, 0, (struct sockaddr *)&clientAddr, &clientAddrLen);
    if (bytes_received < 0) {
        std::cerr << "UDPServer error: receive failed" << std::endl;
        message = "";
        return -1;
    }

    clientURI.ip = inet_ntoa(clientAddr.sin_addr);
    clientURI.port = ntohs(clientAddr.sin_port);

    if (clientURIs.find(clientURI) == clientURIs.end()) {
        clientURIs.insert(clientURI);
    }

    message = buffer.substr(0, bytes_received);
    return 0;
}

void UDPServer::close() {
    ::close(fd);
    isBound = false;
}

std::vector<URI> UDPServer::getClientURIs() {
    return std::vector<URI>(clientURIs.begin(), clientURIs.end());
}