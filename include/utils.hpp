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

    /*
       These system calls are used to access or to change the system
       hostname.  More precisely, they operate on the hostname
       associated with the calling process's UTS namespace.
       - https://man7.org/linux/man-pages/man2/gethostname.2.html
    */
    gethostname(hostname, 1023);

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET; // Use AF_INET to force IPv4
    hints.ai_socktype = SOCK_STREAM;

    /*
       getaddrinfo() returns one or more addrinfo structures as a linked 
       list (in the info struct), each of which contains an Internet 
       address that can be specified in a call to bind(2) or connect(2).
       - https://man7.org/linux/man-pages/man3/getaddrinfo.3.html
    */
    if ((gai_result = getaddrinfo(hostname, NULL, &hints, &info)) != 0) {
        throw std::runtime_error(gai_strerror(gai_result));
    }

    for(p = info; p != NULL; p = p->ai_next) {
        if (p->ai_family == AF_INET) { // IPv4
            struct sockaddr_in *ipv4 = (struct sockaddr_in *)p->ai_addr;
            /*
                This function converts the network address structure src in the
                af address family into a character string. The resulting string
                is copied to the buffer pointed to by dst, which must be a non-
                null pointer.  The caller specifies the number of bytes available
                in this buffer in the argument size.
                - https://man7.org/linux/man-pages/man3/inet_ntop.3.html
            */
            inet_ntop(p->ai_family, &(ipv4->sin_addr), ipstr, sizeof ipstr);
            break;
        }
    }

    freeaddrinfo(info); // free the linked list

    return ipstr;
}

#endif // UTILS_HPP