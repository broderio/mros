#ifndef SERVER_HPP
#define SERVER_HPP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <iostream>
#include <string>

#include "socket/socket.hpp"
#include "socket/connection.hpp"

class ServerSocket : virtual public Socket {
public:
    ServerSocket(int domain, const std::string &address, int port);
    ~ServerSocket();
    ConnectionSocket accept();
    void close();
private:
    sockaddr_in server;
};

#endif // SERVER_HPP