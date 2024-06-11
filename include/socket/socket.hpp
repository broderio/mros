#ifndef SOCKET_HPP
#define SOCKET_HPP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <string>

// Abstract class for socket
class Socket {
protected:
    Socket();
    explicit Socket(int fd);
    virtual ~Socket() = 0;
    int fd;
};

#endif // SOCKET_HPP