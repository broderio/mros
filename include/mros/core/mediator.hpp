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

#include "socket/common.hpp"
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
#include "mros/utils/signalHandler.hpp"
#include "mros/utils/console.hpp"

namespace mros
{
    class Mediator
    {
    public:

        static Mediator &getInstance();

        void init();

        void shutdown();

        void spin();

    private:

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

        struct Service
        {
            URIPair srv;
            std::string inMsgType;
            std::string outMsgType;
        };

        Mediator();

        void run();

        bool ok();

        void registerPublisher(const private_msgs::Register &msg, const URI &clientURI);
        void deregisterPublisher(const private_msgs::Register &msg, const URI &clientURI);

        void registerSubscriber(const private_msgs::Register &msg, const URI &clientURI);
        void deregisterSubscriber(const private_msgs::Register &msg, const URI &clientURI);

        void registerService(const private_msgs::Register &msg, const URI &clientURI);
        void deregisterService(const private_msgs::Register &msg, const URI &clientURI);
        void handleServiceRequest(const std_msgs::String &msg, const URI &clientURI);

        UDPServer server;
        std::map<std::string, Topic> topicMap;
        std::map<std::string, Service> serviceMap;

        bool initialized;
        bool spinning;
        bool shutdownFlag;
    };

}