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
        //std::cerr << "UDPServer error: already open" << std::endl;
        return -1;
    }

    server.sin_family = AF_INET;
    server.sin_port = htons(uri.port);
    if (inet_pton(AF_INET, uri.ip.c_str(), &(server.sin_addr)) <= 0) 
    {
        //std::cerr << "UDPServer error: Invalid address/ Address not supported" << std::endl;
        return -1;
    }

    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        //std::cerr << "UDPServer error: failed to create socket" << std::endl;
        return -1;
    }

    if (nonblocking)
    {
        if (fcntl(fd, F_SETFL, O_NONBLOCK) < 0)
        {
            //std::cerr << "UDPServer error: failed to set non-blocking" << std::endl;
            ::close(fd);
            return -1;
        }
    }

    opened = true;
    return 0;
}

int UDPServer::bind()
{
    if (!opened)
    {
        //std::cerr << "UDPServer error: not open" << std::endl;
        return -1;
    }

    if (bound)
    {
        //std::cerr << "UDPServer error: already bound" << std::endl;
        return -1;
    }

    if (::bind(fd, (struct sockaddr *)&server, sizeof(server)) < 0)
    {
        //perror("UDPServer: bind failed");
        ::close(fd);
        return -1;
    }

    // Get the socket name
    socklen_t len = sizeof(server);
    if (getsockname(fd, (struct sockaddr *)&server, &len) == -1)
    {
        //perror("getsockname");
        ::close(fd);
        return -1;
    }

    bound = true;
    return 0;
}

int UDPServer::sendTo(const std::string &message, const URI &clientURI)
{
    if (!bound)
    {
        //std::cerr << "UDPServer error: not bound" << std::endl;
        return -1;
    }

    sockaddr_in clientAddr;
    clientAddr.sin_family = AF_INET;
    clientAddr.sin_port = htons(clientURI.port);
    if (inet_aton(clientURI.ip.c_str(), &clientAddr.sin_addr) == 0)
    {
        //std::cerr << "UDPServer error: invalid address" << std::endl;
        return -1;
    }

    int bytesSent = sendto(fd, message.c_str(), message.size(), 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr));
    if (bytesSent < 0)
    {
        // //std::cerr << "UDPClient error: send failed" << std::endl;
        //perror("sendto");
        return -1;
    }

    return 0;
}

int UDPServer::receiveFrom(std::string &message, size_t bytes, URI &clientURI)
{
    if (!bound)
    {
        //std::cerr << "UDPServer error: not bound" << std::endl;
        return -1;
    }

    sockaddr_in clientAddr;
    socklen_t clientAddrLen = sizeof(clientAddr);

    std::string buffer(bytes, 0);
    int bytes_received = recvfrom(fd, &buffer[0], bytes, 0, (struct sockaddr *)&clientAddr, &clientAddrLen);
    if (bytes_received < 0)
    {
        message = "";
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return 0;
        }
        else {
            //perror("recvfrom");
            return -1;
        }
    }

    clientURI = URI(inet_ntoa(clientAddr.sin_addr), ntohs(clientAddr.sin_port));

    message = buffer.substr(0, bytes_received);
    return 0;
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

URI UDPServer::getURI()
{
    return URI(inet_ntoa(server.sin_addr), ntohs(server.sin_port));
}