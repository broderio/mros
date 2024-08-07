#include "socket/tcp/client.hpp"

TCPClient::TCPClient()
    : nonblocking(true), connected(false), opened(false) {}

TCPClient::TCPClient(const URI &uri, bool nonblocking)
    : nonblocking(nonblocking), connected(false), opened(false)
{
    open(uri, nonblocking);
}

TCPClient::~TCPClient()
{
    close();
}

int TCPClient::open(const URI &uri, bool nonblocking)
{
    if (opened)
    {
        // std::cerr << "TCPClient error: already open" << std::endl;
        return SOCKET_OPENED;
    }

    // AF_INET specifies that we are using the IPv4 protocol
    server.sin_family = AF_INET;

    // htons() converts the port number from host byte order to network byte order
    server.sin_port = htons(uri.port);

    // inet_aton() converts the address from a string to a binary representation
    if (inet_aton(uri.ip.c_str(), &server.sin_addr) == 0)
    {
        // std::cerr << "TCPClient error: invalid address" << std::endl;
        return SOCKET_INVALID_URI;
    }

    // Create a socket
    // AF_INET specifies that we are using the IPv4 protocol
    // SOCK_STREAM specifies that we are using the TCP protocol
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        // std::cerr << "TCPClient error: failed to create socket" << std::endl;
        return SOCKET_FD_ERROR;
    }

    // Make the socket non-blocking
    if (nonblocking)
    {
        int flags = fcntl(fd, F_GETFL, 0);
        if (flags < 0)
        {
            // std::cerr << "TCPClient error: failed to get flags" << std::endl;
            ::close(fd);
            return SOCKET_FCNTL_ERROR;
        }
        flags |= O_NONBLOCK;
        if (fcntl(fd, F_SETFL, flags) < 0)
        {
            // std::cerr << "TCPClient error: failed to set flags" << std::endl;
            ::close(fd);
            return SOCKET_FCNTL_ERROR;
        }
    }
    this->nonblocking = nonblocking;

    opened = true;
    return SOCKET_OK;
}

int TCPClient::connect()
{
    if (connected)
    {
        return SOCKET_CONNECTED;
    }

    if (::connect(fd, (struct sockaddr *)&server, sizeof(server)) < 0)
    {
        if (errno == EALREADY || errno == EINPROGRESS)
        {
            return SOCKET_OP_IN_PROGRESS;
        }
        if (errno == EISCONN)
        {
            connected = true;
            return SOCKET_OK;
        }
        close();
        return SOCKET_CONNECT_FAILED;
    }
    connected = true;
    return SOCKET_OK;
}

int TCPClient::send(const std::string &message)
{
    // If a connection to the server has not been made yet
    if (!connected)
    {
        return SOCKET_NOT_CONNECTED;
    }

    // Send the message
    if (::send(fd, message.c_str(), message.size(), 0) < 0)
    {
        close();
        return SOCKET_SEND_FAILED;
    }
    return SOCKET_OK;
}

int TCPClient::receive(std::string &message, size_t bytes)
{
    message = "";

    // If a connection to the server has not been made yet
    if (!connected)
    {
        return SOCKET_NOT_CONNECTED;
    }

    // Receive the message, reading up to the specified number of bytes
    std::string buffer(bytes, 0);
    int bytes_received = ::recv(fd, &buffer[0], bytes, 0);
    if (bytes_received < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            // No data available to read
            return SOCKET_OP_WOULD_BLOCK;
        }
        else
        {
            close();
            return SOCKET_RECV_FAILED;
        }
    }

    message = buffer.substr(0, bytes_received);
    return SOCKET_OK;
}

void TCPClient::close()
{
    ::close(fd);
}

bool TCPClient::isOpen() const
{
    return opened;
}

bool TCPClient::isConnected() const
{
    return connected;
}

bool TCPClient::isNonblocking() const
{
    return nonblocking;
}

int TCPClient::getServerURI(URI &uri)
{
    if (!connected)
    {
        return SOCKET_NOT_CONNECTED;
    }
    
    uri = URI(inet_ntoa(server.sin_addr), ntohs(server.sin_port));
    return SOCKET_OK;
}