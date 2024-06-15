#ifndef TCP_CLIENT_HPP
#define TCP_CLIENT_HPP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>
#include <string>

#include "utils.hpp"

class TCPClient {
public:

    TCPClient(const std::string &address, int port, bool nonBlocking = 1);

    ~TCPClient();

    int connect();

    int send(const std::string& message);

    int receive(std::string& message, size_t bytes);

    void close();

    bool isNonBlocking();

    URI getServerURI();

private:

    int fd;

    bool isConnected;

    bool nonBlocking;

    sockaddr_in server;

};

#endif // TCP_CLIENT_HPP