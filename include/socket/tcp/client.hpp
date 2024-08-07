#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>
#include <string>

#include "socket/common.hpp"

#include "utils.hpp"

class TCPClient {
public:

    TCPClient();

    TCPClient(const TCPClient &other) = delete;

    TCPClient(const URI &uri, bool nonblocking = true);

    ~TCPClient();

    int open(const URI &uri, bool nonblocking = true);

    int connect();

    int send(const std::string& message);

    int receive(std::string& message, size_t bytes);

    void close();

    bool isOpen() const;

    bool isConnected() const;

    bool isNonblocking() const;

    int getServerURI(URI &uri); 

private:

    int fd;

    bool connected;

    bool opened;

    bool nonblocking;

    sockaddr_in server;

};