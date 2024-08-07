#include "socket/tcp/server.hpp"

TCPServer::TCPServer()
    : nonblocking(false), maxConnections(0), numConnections(0), opened(false), bound(false), listening(false) {}

TCPServer::TCPServer(const URI &uri, bool nonblocking, int maxConnections)
    : nonblocking(nonblocking), maxConnections(maxConnections), numConnections(0), opened(false), bound(false), listening(false)
{
    open(uri, nonblocking);
}

TCPServer::~TCPServer()
{
    close();
}

int TCPServer::open(const URI &uri, bool nonblocking, int maxConnections)
{
    if (opened)
    {
        //std::cerr << "TCPServer error: already open" << std::endl;
        return -1;
    }

    // AF_INET specifies that we are using the IPv4 protocol
    server.sin_family = AF_INET;

    // htons() converts the port number from host byte order to network byte order
    server.sin_port = htons(uri.port);

    if (inet_pton(AF_INET, uri.ip.c_str(), &(server.sin_addr)) <= 0)
    {
        return SOCKET_INVALID_URI;
    }

    // Create a socket
    // AF_INET specifies that we are using the IPv4 protocol
    // SOCK_STREAM specifies that we are using the TCP protocol
    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        return SOCKET_FD_ERROR;
    }

    // Make the socket non-blocking
    if (nonblocking)
    {
        int flags = fcntl(fd, F_GETFL, 0);
        if (flags < 0)
        {
            close();
            return SOCKET_FCNTL_ERROR;
        }
        flags |= O_NONBLOCK;
        if (fcntl(fd, F_SETFL, flags) < 0)
        {
            close();
            return SOCKET_FCNTL_ERROR;
        }
    }
    this->nonblocking = nonblocking;

    maxConnections = maxConnections;
    opened = true;
    return SOCKET_OK;
}

int TCPServer::bind()
{
    if (!opened)
    {
        return SOCKET_NOT_OPENED;
    }

    if (bound)
    {
        return SOCKET_BOUND;
    }

    // Bind the socket to the address and port
    if (::bind(fd, (struct sockaddr *)&server, sizeof(server)) < 0)
    {
        close();
        return SOCKET_BIND_FAILED;
    }
    bound = true;

    // Update the server information
    socklen_t len = sizeof(server);
    if (getsockname(fd, (struct sockaddr *)&server, &len) == -1)
    {
        //perror("getsockname");
        close();
        return SOCKET_GETSOCKNAME_ERROR;
    }

    return SOCKET_OK;
}

int TCPServer::listen()
{
    if (!bound)
    {
        return SOCKET_NOT_BOUND;
    }

    if (listening)
    {
        return SOCKET_LISTENING;
    }

    // Listen for incoming connections
    if (::listen(fd, maxConnections) < 0)
    {
        close();
        return SOCKET_LISTEN_FAILED;
    }
    listening = true;

    return SOCKET_OK;
}

int TCPServer::accept(std::shared_ptr<TCPConnection> &connection)
{
    if (!listening)
    {
        return SOCKET_NOT_LISTENING;
    }

    // If maximum number of connections has been reached
    if (numConnections >= maxConnections)
    {
        return SOCKET_MAX_CONNECTIONS;
    }

    // Client information
    struct sockaddr_in client;
    socklen_t client_len = sizeof(client);

    // Accept a connection
    int client_fd = ::accept(fd, (struct sockaddr *)&client, &client_len);

    // If the accept call fails
    if (client_fd < 0)
    {
        // If the error is EWOULDBLOCK or EAGAIN, there are no pending connections
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
            return SOCKET_OP_WOULD_BLOCK;
        }
        // Otherwise, there was an error
        else
        {
            close();
            return SOCKET_ACCEPT_FAILED;
        }
    }

    // Make the socket non-blocking
    if (nonblocking)
    {
        int flags = fcntl(client_fd, F_GETFL, 0);
        if (flags < 0)
        {
            ::close(client_fd);
            return SOCKET_FCNTL_ERROR;
        }
        flags |= O_NONBLOCK;
        if (fcntl(client_fd, F_SETFL, flags) < 0)
        {
            ::close(client_fd);
            return SOCKET_FCNTL_ERROR;
        }
    }

    std::shared_ptr<TCPConnection> ptr(new TCPConnection(client_fd, client, nonblocking));
    connection = ptr;
    numConnections++;
    return SOCKET_OK;
}

void TCPServer::close()
{
    ::close(fd);
    opened = false;
    bound = false;
    listening = false;
}

bool TCPServer::isOpen() const
{
    return opened;
}

bool TCPServer::isBound() const
{
    return bound;
}

bool TCPServer::isListening() const
{
    return listening;
}

bool TCPServer::isNonblocking() const
{
    return nonblocking;
}

int TCPServer::getURI(URI &uri)
{
    if (!bound)
    {
        return SOCKET_NOT_BOUND;
    }

    uri = URI(inet_ntoa(server.sin_addr), ntohs(server.sin_port));
    return SOCKET_OK;
}

TCPConnection::TCPConnection()
{
    fd = -1;
    opened = false;
}

TCPConnection::TCPConnection(int fd, sockaddr_in client, bool nonblocking)
    : fd(fd), client(client), nonblocking(nonblocking), opened(true) {}

TCPConnection::~TCPConnection()
{
    close();
}

int TCPConnection::send(const std::string &message)
{
    // If the connection is closed (i.e. this object was not created by a server)
    if (!opened)
    {
        return SOCKET_NOT_OPENED;
    }

    // Send the message
    if (::send(fd, message.c_str(), message.size(), 0) < 0)
    {
        close();
        return SOCKET_SEND_FAILED;
    }
    return SOCKET_OK;
}

int TCPConnection::receive(std::string &message, size_t bytes)
{
    message = "";

    // If the connection is closed (i.e. this object was not created by a server)
    if (!opened)
    {
        return SOCKET_NOT_OPENED;
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
            return SOCKET_RECV_FAILED;
        }
    }

    message = buffer.substr(0, bytes_received);
    return SOCKET_OK;
}

void TCPConnection::close()
{
    ::close(fd);
    opened = false;
}

bool TCPConnection::isOpen() const
{
    return opened;
}

bool TCPConnection::isNonblocking() const
{
    return nonblocking;
}

int TCPConnection::getClientURI(URI &uri)
{
    if (!opened)
    {
        return SOCKET_NOT_OPENED;
    }

    uri = URI(inet_ntoa(client.sin_addr), ntohs(client.sin_port));
    return SOCKET_OK;
}