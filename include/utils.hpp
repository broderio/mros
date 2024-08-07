#pragma once 

#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <cmath>

#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "unistd.h"

#include <sys/types.h>
#include <ifaddrs.h>
#include <string.h>

#include <chrono>
#include <thread>

struct URI {
    URI();
    URI(const std::string& ip, int port);
    
    bool operator<(const URI& other) const;
    bool operator==(const URI& other) const;
    bool operator!=(const URI& other) const;

    std::string toString() const;
    
    std::string ip;
    int port;
};

std::ostream &operator<<(std::ostream &os, const URI &uri);

void sleep(int milliseconds);

struct DateTime {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int millisecond;

    std::string toString() const;
};

DateTime getDateTime();

int64_t getTimeMilli();

int64_t getTimeNano();

float degToRad(float deg);

float radToDeg(float rad);

std::string getLocalIP();

std::string addTab(const std::string &str, int tabCount);