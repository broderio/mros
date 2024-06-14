#ifndef CLIENT_HPP
#define CLIENT_HPP

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>

#include <iostream>
#include <string>

class TCPClient {
public:

    TCPClient(const std::string &address, int port);

    ~TCPClient();

    int connect();

    int send(const std::string& message);

    int receive(std::string& message, size_t bytes);

    void close();

private:

    int fd;

    bool isConnected;

    sockaddr_in server;

};

#endif // CLIENT_HPP