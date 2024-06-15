#ifndef UTILS_HPP
#define UTILS_HPP

#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "unistd.h"

#include <chrono>
#include <thread>

struct URI {
    URI();
    URI(const std::string& ip, int port);
    bool operator<(const URI& other) const;
    bool operator==(const URI& other) const;
    std::string toString() const;
    
    std::string ip;
    int port;
};

std::ostream &operator<<(std::ostream &os, const URI &uri);

void sleep(int milliseconds);

int getTimeMs();

std::string getIPAddr();

#endif // UTILS_HPP