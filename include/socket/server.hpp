#ifndef SERVER_HPP
#define SERVER_HPP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <iostream>
#include <string>

class TCPConnection {
    friend class TCPServer;
public:
    TCPConnection();

    ~TCPConnection();
    
    int send(const std::string& message);
    
    int receive(std::string& message, size_t bytes);
    
    void close();

    bool isNonBlocking();

private:
    
    int fd;
    
    bool isOpen;

    bool nonBlocking;

};

class TCPServer {
public:
    
    TCPServer(const std::string &address, int port, bool nonBlocking = 1, int maxConnections = 10);
    
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

#endif // SERVER_HPP