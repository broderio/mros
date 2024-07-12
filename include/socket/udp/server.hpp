#ifndef UDP_SERVER_HPP
#define UDP_SERVER_HPP

#include <iostream>
#include <string>
#include <set>

#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>

#include "utils.hpp"

class UDPServer
{
public:
    UDPServer();

    UDPServer(int port, bool nonBlocking = true);

    ~UDPServer();

    int open(int port, bool nonBlocking = true);

    int bind();

    int sendTo(const std::string &message, const URI &clientURI);

    int receiveFrom(std::string &message, size_t bytes, URI &clientURI);

    void close();

    bool isOpen() const;

    bool isBound() const;

    bool isNonblocking() const;

    URI getURI();

private:
    int fd;

    bool opened;

    bool bound;

    bool nonblocking;
    
    sockaddr_in server;
};

#endif // UDP_SERVER_HPP