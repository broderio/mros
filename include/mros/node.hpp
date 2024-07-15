#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <queue>
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

#include "mros/private_msgs/notify.hpp"
#include "mros/private_msgs/register.hpp"
#include "mros/private_msgs/request.hpp"
#include "mros/private_msgs/disconnect.hpp"
#include "mros/private_msgs/uri.hpp"
#include "mros/private_msgs/response.hpp"

#include "mros/common.hpp"
#include "mros/subscriber.hpp"
#include "mros/publisher.hpp"
#include "mros/signalHandler.hpp"
#include "mros/console.hpp"

namespace mros
{

    class Node
    {
    public:
        Node(const std::string &name, const URI &coreURI);

        ~Node();

        void spin(bool detach=false);

        void spinOnce();

        void shutdown();

        bool ok();

        template <typename MsgType>
        std::shared_ptr<Publisher> advertise(const std::string &topic, const size_t &queueSize)
        {

            std::string msgType = typeid(MsgType).name();
            std::shared_ptr<Publisher> pubPtr = Publisher::create(topic, queueSize, msgType);

            if (!contactMediator(topic, msgType, pubPtr->publicServer.getURI(), CORE_TOPICS::PUB_REGISTER))
            {
                return nullptr;
            }

            publishers[topic].insert(pubPtr);
            return pubPtr;
        }

        template <typename MsgType>
        std::shared_ptr<Subscriber> subscribe(const std::string &topic, const size_t &queueSize, std::function<void(const MsgType &)> callback)
        {
            // Create Subscriber object
            std::shared_ptr<Subscriber> subPtr = Subscriber::create(topic, queueSize, callback);

            // Register subscriber with the Mediator
            if (!contactMediator(topic, typeid(MsgType).name(), subPtr->publicServer.getURI(), CORE_TOPICS::SUB_REGISTER))
            {
                return nullptr;
            }

            // Add Subscriber object to the map
            subscribers[topic].insert(subPtr);

            return subPtr;
        }

    private:
        void run();

        void runOnce();

        bool contactMediator(const std::string &topic, const std::string &msgType, const URI &publicURI, const uint16_t &id);

        void handlePubNotify(const private_msgs::Notify &notify);
        void handleSubNotify(const private_msgs::Notify &notify);
        void handlePubDisconnect(const private_msgs::Disconnect &disconnect);
        void handleSubDisconnect(const private_msgs::Disconnect &disconnect);

        SignalHandler signalHandler;

        std::string name;
        URI coreURI;

        bool spinning;
        bool shutdownFlag;

        std::map<std::string, std::set<std::shared_ptr<Publisher>>> publishers;   // Map of topic names to Publisher objects
        std::map<std::string, std::set<std::shared_ptr<Subscriber>>> subscribers; // Map of topic names to Subscriber objects

        UDPClient client; // For talking to the Mediator
    };

}