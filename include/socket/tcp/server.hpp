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

    bool isOpen() const; 

    bool isNonblocking() const;

    URI getClientURI();

private:
    
    int fd;

    sockaddr_in client;
    
    bool opened;

    bool nonblocking;

};

class TCPServer {
public:
    TCPServer();

    TCPServer(int port, bool nonblocking = 1, int maxConnections = 10);
    
    ~TCPServer();

    int open(int port, bool nonblocking = 1, int maxConnections = 10);

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

#endif // TCP_SERVER_HPP