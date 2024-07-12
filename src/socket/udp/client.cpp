#include "socket/udp/client.hpp"

UDPClient::UDPClient(bool nonblocking)
    : opened(false), nonblocking(nonblocking)
{
    open(nonblocking);
}

UDPClient::~UDPClient()
{
    close();
}

int UDPClient::open(bool nonblocking) 
{
    if (opened)
    {
        std::cerr << "UDPClient error: already open" << std::endl;
        return -1;
    }

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        std::cerr << "UDPClient error: socket creation failed" << std::endl;
        return -1;
    }

    if (nonblocking)
    {
        if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0)
        {
            std::cerr << "UDPClient error: failed to set non-blocking" << std::endl;
            ::close(fd);
            return -1;
        }
    }
    this->nonblocking = nonblocking;
    this->opened = true;
    return 0;
}

int UDPClient::sendTo(const std::string &message, const URI &serverURI)
{
    if (!opened)
    {
        std::cerr << "UDPClient error: not open" << std::endl;
        return -1;
    }

    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(serverURI.port);

    if (inet_aton(serverURI.ip.c_str(), &server.sin_addr) == 0)
    {
        std::cerr << "UDPClient error: invalid server address" << std::endl;
        return -1;
    }

    int bytesSent = sendto(fd, message.c_str(), message.size(), 0, (struct sockaddr *)&server, sizeof(server));
    if (bytesSent < 0)
    {
        // std::cerr << "UDPClient error: send failed" << std::endl;
        perror("sendto");
        return -1;
    }

    return 0;
}

int UDPClient::receiveFrom(std::string &message, size_t bytes, URI &serverURI)
{
    if (!opened)
    {
        std::cerr << "UDPClient error: not open" << std::endl;
        return -1;
    }

    std::string buffer(bytes, 0);

    struct sockaddr_in server;
    socklen_t serverLen = sizeof(server);
    int bytes_received = recvfrom(fd, &buffer[0], bytes, 0, (struct sockaddr *)&server, &serverLen);
    if (bytes_received < 0)
    {
        message = "";
        if (errno == EAGAIN | errno == EWOULDBLOCK)
        {
            return 0;
        }
        else {
            std::cerr << "UDPClient error: receive failed" << std::endl;
            return -1;
        }
    }

    serverURI.ip = inet_ntoa(server.sin_addr);
    serverURI.port = ntohs(server.sin_port);

    message = buffer.substr(0, bytes_received);
    return 0;
}

void UDPClient::close()
{
    ::close(fd);
    opened = false;
    nonblocking = true; // Set to default
}

bool UDPClient::isOpen() const
{
    return opened;
}

bool UDPClient::isNonblocking() const
{
    return nonblocking;
}