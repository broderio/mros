#pragma once

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

#include "socket/common.hpp"

#include "utils.hpp"

class UDPServer
{
public:
    UDPServer();

    UDPServer(const URI &uri, bool nonBlocking = true);

    UDPServer(const UDPServer &other) = delete;

    ~UDPServer();

    int open(const URI &uri, bool nonBlocking = true);

    int bind();

    int sendTo(const std::string &message, const URI &clientURI);

    int receiveFrom(std::string &message, size_t bytes, URI &clientURI);

    void close();

    bool isOpen() const;

    bool isBound() const;

    bool isNonblocking() const;

    int getURI(URI &uri);

private:
    int fd;

    bool opened;

    bool bound;

    bool nonblocking;
    
    sockaddr_in server;
};