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
        return SOCKET_OPENED;
    }

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        return SOCKET_FD_ERROR;
    }

    if (nonblocking)
    {
        if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0)
        {
            close();
            return SOCKET_FCNTL_ERROR;
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
        return SOCKET_NOT_OPENED;
    }

    struct sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(serverURI.port);

    if (inet_aton(serverURI.ip.c_str(), &server.sin_addr) == 0)
    {
        return SOCKET_INVALID_URI;
    }

    int bytesSent = sendto(fd, message.c_str(), message.size(), 0, (struct sockaddr *)&server, sizeof(server));
    if (bytesSent < 0)
    {
        close();
        return SOCKET_SEND_FAILED;
    }

    return 0;
}

int UDPClient::receiveFrom(std::string &message, size_t bytes, URI &serverURI)
{
    message = "";

    if (!opened)
    {
        return SOCKET_NOT_OPENED;
    }

    std::string buffer(bytes, 0);

    struct sockaddr_in server;
    socklen_t serverLen = sizeof(server);
    int bytes_received = recvfrom(fd, &buffer[0], bytes, 0, (struct sockaddr *)&server, &serverLen);
    if (bytes_received < 0)
    {
        if (errno == EAGAIN | errno == EWOULDBLOCK)
        {
            return SOCKET_OP_WOULD_BLOCK;
        }
        else
        {
            close();
            return SOCKET_RECV_FAILED;
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