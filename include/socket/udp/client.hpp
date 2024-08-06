#pragma once

#include <iostream>
#include <string>

#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>

#include "utils.hpp"

class UDPClient
{
public:
    UDPClient(bool nonblocking = true);

    ~UDPClient();

    int open(bool nonblocking = true);

    int sendTo(const std::string &message, const URI &serverURI);

    int receiveFrom(std::string &message, size_t bytes, URI &serverURI);

    void close();

    bool isOpen() const;

    bool isNonblocking() const;

private:
    int fd;

    bool opened;

    bool nonblocking;
};