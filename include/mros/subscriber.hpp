#ifndef SUBSCRIBER_HPP
#define SUBSCRIBER_HPP

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <thread>
#include <chrono>

#include "utils.hpp"

#include "socket/tcp/client.hpp"
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

class ISubscriber {
public:
    virtual ~ISubscriber() = default;

private:
    virtual void invokeCallback(const std::string& msg) = 0;
};

template<typename T>
class Subscriber : public ISubscriber {
public:
    Subscriber(std::function<void(const T&)> callback, const std::string& topic, const size_t& queueSize);
    ~Subscriber();

private:
    std::vector<TCPClient> pubClients; // For receiving public messages from Publishers

    UDPServer server; // For receiving private messages from the Mediator and Publishers
    UDPClient mediatorClient; // For sending private messages to the Mediator

    std::string topic;
    std::string msgType;
    std::string hostName;
    size_t queueSize;

    std::function<void(const T&)> callback;

    void invokeCallback(const std::string& msg) override;
};

}

#endif // SUBSCRIBER_HPP