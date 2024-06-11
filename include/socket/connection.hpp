#ifndef CONNECTION_HPP
#define CONNECTION_HPP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <string>

#include <socket/socket.hpp>

class ConnectionSocket : virtual public Socket {
public:
    explicit ConnectionSocket(int fd);
    ~ConnectionSocket();
    void send(const std::string& message);
    std::string receive();
    void close();
};

#endif // CONNECTION_HPP