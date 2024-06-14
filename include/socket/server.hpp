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

private:
    
    int fd;
    
    bool isOpen;
    
};

class TCPServer {
public:
    
    TCPServer(const std::string &address, int port, int max_connections = 10);
    
    ~TCPServer();

    int accept(TCPConnection& connection);
    
    void close();

private:
    
    int fd;

    int max_connections;

    int num_connections;
    
    sockaddr_in server;

};

#endif // SERVER_HPP