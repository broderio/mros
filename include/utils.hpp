#ifndef UTILS_HPP
#define UTILS_HPP

#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "unistd.h"

#include <chrono>
#include <thread>

void sleep(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

int getTimeMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string getIPAddr() {
    struct addrinfo hints, *info, *p;
    int gai_result;
    char hostname[1024];
    char ipstr[INET_ADDRSTRLEN];

    hostname[1023] = '\0';
    gethostname(hostname, 1023);

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET; // Use AF_INET to force IPv4
    hints.ai_socktype = SOCK_STREAM;

    if ((gai_result = getaddrinfo(hostname, NULL, &hints, &info)) != 0) {
        throw std::runtime_error(gai_strerror(gai_result));
    }

    for(p = info; p != NULL; p = p->ai_next) {
        if (p->ai_family == AF_INET) { // IPv4
            struct sockaddr_in *ipv4 = (struct sockaddr_in *)p->ai_addr;
            // convert the IP to a string
            inet_ntop(p->ai_family, &(ipv4->sin_addr), ipstr, sizeof ipstr);
            break;
        }
    }

    freeaddrinfo(info); // free the linked list

    return ipstr;
}

#endif // UTILS_HPP