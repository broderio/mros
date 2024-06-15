#ifndef TCP_SERVER_HPP
#define TCP_SERVER_HPP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <iostream>
#include <string>

#include "utils.hpp"

class TCPConnection {
    friend class TCPServer;
public:
    TCPConnection();

    ~TCPConnection();
    
    int send(const std::string& message);
    
    int receive(std::string& message, size_t bytes);
    
    void close();

    bool isNonBlocking();

    URI getClientURI();

private:
    
    int fd;

    sockaddr_in client;
    
    bool isOpen;

    bool nonBlocking;

};

class TCPServer {
public:
    
    TCPServer(int port, bool nonBlocking = 1, int maxConnections = 10);
    
    ~TCPServer();

    int accept(TCPConnection& connection);
    
    void close();

    bool isNonBlocking();

private:
    
    int fd;

    int maxConnections;

    int numConnections;

    bool nonBlocking;
    
    sockaddr_in server;

};

#endif // TCP_SERVER_HPP