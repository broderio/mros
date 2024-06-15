#ifndef UDP_CLIENT_HPP
#define UDP_CLIENT_HPP

#include <iostream>
#include <string> 

#include <sys/types.h> 
#include <arpa/inet.h> 
#include <sys/socket.h> 
#include <netinet/in.h> 
#include <unistd.h> 
#include <stdlib.h>

class UDPClient {
public:
    UDPClient(const std::string &address, int port);

    ~UDPClient();

    int connect();

    int sendTo(const std::string& message);

    int receiveFrom(std::string& message, size_t bytes);

    void close();

private:
    
        int fd;
    
        sockaddr_in serverAddr;
    
        bool isConnected;
};

#endif // UDP_CLIENT_HPP