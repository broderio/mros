#include "socket/udp/server.hpp"

UDPServer::UDPServer()
    : bound(false), opened(false), nonblocking(true)
{
}

UDPServer::UDPServer(const URI &uri, bool nonblocking)
    : bound(false), opened(false), nonblocking(nonblocking)
{
    open(uri, nonblocking);
}

UDPServer::~UDPServer()
{
    close();
}

int UDPServer::open(const URI &uri, bool nonblocking)
{
    if (opened)
    {
        return SOCKET_OPENED;
    }

    server.sin_family = AF_INET;
    server.sin_port = htons(uri.port);
    if (inet_pton(AF_INET, uri.ip.c_str(), &(server.sin_addr)) <= 0)
    {
        return SOCKET_INVALID_URI;
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

    opened = true;
    return SOCKET_OK;
}

int UDPServer::bind()
{
    if (!opened)
    {
        return SOCKET_NOT_OPENED;
    }

    if (bound)
    {
        return SOCKET_BOUND;
    }

    if (::bind(fd, (struct sockaddr *)&server, sizeof(server)) < 0)
    {
        close();
        return SOCKET_BIND_FAILED;
    }

    // Get the socket name
    socklen_t len = sizeof(server);
    if (getsockname(fd, (struct sockaddr *)&server, &len) == -1)
    {
        close();
        return SOCKET_GETSOCKNAME_ERROR;
    }

    bound = true;
    return SOCKET_OK;
}

int UDPServer::sendTo(const std::string &message, const URI &clientURI)
{
    if (!bound)
    {
        return SOCKET_NOT_BOUND;
    }

    sockaddr_in clientAddr;
    clientAddr.sin_family = AF_INET;
    clientAddr.sin_port = htons(clientURI.port);
    if (inet_aton(clientURI.ip.c_str(), &clientAddr.sin_addr) == 0)
    {
        return SOCKET_INVALID_URI;
    }

    int bytesSent = sendto(fd, message.c_str(), message.size(), 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr));
    if (bytesSent < 0)
    {
        close();
        return SOCKET_SEND_FAILED;
    }

    return SOCKET_OK;
}

int UDPServer::receiveFrom(std::string &message, size_t bytes, URI &clientURI)
{
    message = "";

    if (!bound)
    {
        return SOCKET_NOT_BOUND;
    }

    sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);

    std::string buffer(bytes, 0);
    int bytes_received = recvfrom(fd, &buffer[0], bytes, 0, (struct sockaddr *)&clientAddr, &clientAddrLen);
    if (bytes_received < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return SOCKET_OP_WOULD_BLOCK;
        }
        else
        {
            close();
            return SOCKET_RECV_FAILED;
        }
    }

    clientURI = URI(inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port));

    message = buffer.substr(0, bytes_received);
    return SOCKET_OK;
}

void UDPServer::close()
{
    ::close(fd);
    bound = false;
}

bool UDPServer::isOpen() const
{
    return opened;
}

bool UDPServer::isBound() const
{
    return bound;
}

bool UDPServer::isNonblocking() const
{
    return nonblocking;
}

int UDPServer::getURI(URI &uri)
{
    if (!bound)
    {
        return SOCKET_NOT_BOUND;
    }
    uri = URI(inet_ntoa(server.sin_addr), ntohs(server.sin_port));
    return SOCKET_OK;
}