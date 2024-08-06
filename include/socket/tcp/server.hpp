#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <iostream>
#include <string>

#include "utils.hpp"

class TCPConnection;

class TCPServer {
public:
    TCPServer();

    TCPServer(const URI &uri, bool nonblocking = 1, int maxConnections = 10);
    
    ~TCPServer();

    int open(const URI &uri, bool nonblocking = 1, int maxConnections = 10);

    int bind();

    int listen();

    int accept(TCPConnection& connection);
    
    void close();

    bool isOpen() const;

    bool isBound() const;

    bool isListening() const;

    bool isNonblocking() const;

    URI getURI();

private:
    
    int fd;

    int maxConnections;

    int numConnections;

    bool opened;

    bool bound;

    bool listening;

    bool nonblocking;
    
    sockaddr_in server;
};

class TCPConnection {
    friend class TCPServer;
public:
    TCPConnection();

    ~TCPConnection();
    
    int send(const std::string& message);
    
    int receive(std::string& message, size_t bytes);
    
    void close();

    bool isOpen() const; 

    bool isNonblocking() const;

    URI getClientURI();

private:

    TCPConnection(int fd, sockaddr_in client, bool nonblocking);
    
    int fd;

    sockaddr_in client;
    
    bool opened;

    bool nonblocking;

};