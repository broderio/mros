#pragma once

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
#include "mros/private_msgs/notify.hpp"
#include "mros/private_msgs/response.hpp"
#include "mros/private_msgs/uri.hpp"

#include "mros/common.hpp"
#include "mros/signalHandler.hpp"

namespace mros
{
    class Mediator
    {
    public:
        Mediator();
        ~Mediator();

        void spin();

    private:
        UDPServer server;

        struct URIPair 
        {
            URI clientURI; // Node client URI
            URI serverURI; // Publisher/Subscriber server URI
            bool operator<(const URIPair &other) const;
        };

        struct Topic
        {
            std::set<URIPair> pubs;
            std::set<URIPair> subs;
            std::string msgType;
        };

        std::map<std::string, Topic> topicMap;

        SignalHandler signalHandler;

        void run();
        void registerPublisher(const private_msgs::Register &msg, const URI &clientURI);
        void registerSubscriber(const private_msgs::Register &msg, const URI &clientURI);
        void deregisterPublisher(const private_msgs::Register &msg, const URI &clientURI);
        void deregisterSubscriber(const private_msgs::Register &msg, const URI &clientURI);
        void notifySubscribers(const std::string &topic);
    };

}