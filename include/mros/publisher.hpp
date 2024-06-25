#ifndef PUBLISHER_HPP
#define PUBLISHER_HPP

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
#include "socket/udp/client.hpp"

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"
#include "messages/std_msgs/time.hpp"
#include "messages/std_msgs/string.hpp"

#include "mros/private_msgs/register.hpp"
#include "mros/private_msgs/request.hpp"
#include "mros/private_msgs/disconnect.hpp"
#include "mros/private_msgs/uri.hpp"

#include "mros/common.hpp"

namespace mros {

class IPublisher {
public:
    virtual ~IPublisher() = default;

    virtual void publish(const std::string& msg) = 0;
};

template<typename T>
class Publisher : public IPublisher {
public:
    Publisher(const std::string& topic, const size_t& queueSize);
    ~Publisher();

    void publish(const T& msg) override; 

private:
    TCPServer subServer; // For accepting connections from Subscribers
    std::vector<TCPConnection> subConnections; // For sending public messages to Subscribers

    UDPServer server; // For receiving private messages from Subscribers and the Mediator
    UDPClient mediatorClient; // For sending private messages to the Mediator

    std::string topic;
    std::string msgType;
    std::string hostName;
};

}

#endif // PUBLISHER_HPP