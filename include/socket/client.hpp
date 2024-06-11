#ifndef CLIENT_HPP
#define CLIENT_HPP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <string>

#include "socket/socket.hpp"

class ClientSocket : virtual public Socket {
public:
    ClientSocket(int domain, const std::string &address, int port);
    ~ClientSocket();
    void connect();
    void send(const std::string& message);
    std::string receive();
    void close();
private:
    sockaddr_in server;
};

#endif // CLIENT_HPP