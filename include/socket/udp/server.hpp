#ifndef UDP_SERVER_HPP
#define UDP_SERVER_HPP

#include <iostream>
#include <string> 
#include <set>

#include <sys/types.h> 
#include <arpa/inet.h> 
#include <sys/socket.h> 
#include <netinet/in.h> 
#include <unistd.h> 
#include <stdlib.h>

#include "utils.hpp"

class UDPServer {
public:
    UDPServer(int port);
    
    ~UDPServer();
    
    int bind();
    
    int sendTo(const std::string& message, const URI &clientURI);
    
    int receiveFrom(std::string& message, size_t bytes, URI &clientURI);
    
    void close();

    std::vector<URI> getClientURIs();

private:

    int fd;

    sockaddr_in serverAddr;
    std::set<URI> clientURIs;

    bool isBound;

};

#endif // UDP_SERVER_HPP