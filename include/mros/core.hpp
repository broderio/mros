#ifndef CORE_HPP
#define CORE_HPP

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <thread>
#include <chrono>

#include <socket/tcp/server.hpp>

class Core {
public:
    Core();
    ~Core();
    void run();
    void stop();
private: 
    std::map<std::string, std::set<int>> publishers; // publisher_name -> [socket_fd]
    std::map<std::string, std::set<int>> subscribers; // subscriber_name -> [socket_fd]
    std::map<std::string, std::vector<std::string>> topic_publishers; // topic_name -> [publisher_name]
    std::map<std::string, std::vector<std::string>> topic_subscribers; // topic_name -> [subcriber_name]
    std::map<std::string, std::string> topic_types; // topic_name -> topic_type
};

#endif // CORE_HPP