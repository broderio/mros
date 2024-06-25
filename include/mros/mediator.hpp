#ifndef MROS_MEDIATOR_HPP
#define MROS_MEDIATOR_HPP

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <thread>
#include <chrono>

#include "utils.hpp"

#include "socket/tcp/server.hpp"
#include "socket/udp/server.hpp"

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"
#include "messages/std_msgs/time.hpp"
#include "messages/std_msgs/string.hpp"

#include "mros/private_msgs/register.hpp"
#include "mros/private_msgs/request.hpp"
#include "mros/private_msgs/disconnect.hpp"
#include "mros/private_msgs/uri.hpp"

#include "mros/common.hpp"

#define MEDIATOR_PORT_NUM   11311

namespace mros {

struct Topic {
    std::set<URI> publishers;
    std::set<URI> subscribers;
    std::string msgType;
};

class Mediator {
public:
    Mediator();
    ~Mediator();

    void spin();
private:
    UDPServer server;
    std::map<std::string, Topic> topicMap;

    void run();
    void parseRegisterMessage(const private_msgs::Register& msg);
    void parseDeregisterMessage(const private_msgs::Register& msg);
    void notifySubscribers(const std::string& topic);
};

}

#endif // MROS_MEDIATOR_HPP