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
#include <mutex>

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

#include "mros/utils/signalHandler.hpp"
#include "mros/utils/console.hpp"

#include "mros/core/subscriber.hpp"
#include "mros/core/publisher.hpp"
#include "mros/core/service.hpp"

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
            static_assert(std::is_base_of<IMessage, MsgType>::value, "MsgType must inherit from IMessage");

            std::string msgType = typeid(MsgType).name();
            std::shared_ptr<Publisher> pubPtr = Publisher::create(topic, queueSize, msgType);

            registerPublisher(pubPtr);

            Console::log(LogLevel::INFO, "Advertising topic: \"" + topic + "\"");

            publishers[topic].insert(pubPtr);
            return pubPtr;
        }

        template <typename MsgType>
        std::shared_ptr<Subscriber> subscribe(const std::string &topic, const size_t &queueSize, std::function<void(const MsgType &)> callback)
        {
            static_assert(std::is_base_of<IMessage, MsgType>::value, "MsgType must inherit from IMessage");

            std::shared_ptr<Subscriber> subPtr = Subscriber::create(topic, queueSize, callback);

            registerSubscriber(subPtr);

            Console::log(LogLevel::INFO, "Subscribed to topic: \"" + topic + "\"");

            subscribers[topic].insert(subPtr);

            return subPtr;
        }

        template <typename InMsgType, typename OutMsgType>
        std::shared_ptr<Service> advertiseService(const std::string &service, std::function<void(const InMsgType &request, OutMsgType &response)> callback)
        {
            static_assert(std::is_base_of<IMessage, InMsgType>::value, "InMsgType must inherit from IMessage");
            static_assert(std::is_base_of<IMessage, OutMsgType>::value, "OutMsgType must inherit from IMessage");

            std::shared_ptr<Service> srvPtr = Service::create<InMsgType, OutMsgType>(service, callback);

            Console::log(LogLevel::INFO, "Advertising service: " + service);
            registerService(srvPtr);

            services[service].insert(srvPtr);

            return srvPtr;
        }

        template <typename InMsgType, typename OutMsgType>
        bool callService(const std::string &service, const InMsgType &request, OutMsgType &response)
        {
            static_assert(std::is_base_of<IMessage, InMsgType>::value, "InMsgType must inherit from IMessage");
            static_assert(std::is_base_of<IMessage, OutMsgType>::value, "OutMsgType must inherit from IMessage");

            URI serviceURI;
            std::string inMsgType = typeid(InMsgType).name();
            std::string outMsgType = typeid(OutMsgType).name();
            if (!requestServiceAddress(service, serviceURI, inMsgType, outMsgType))
            {
                return false;
            }

            std::string requestStr = Parser::encode(request, 0);

            std::string responseStr = callService(serviceURI, requestStr);

            if (responseStr.empty())
            {
                return false;
            }

            return Parser::decode(responseStr, response);
        }

    private:
        void run();

        void runOnce();

        void registerSubscriber(std::shared_ptr<Subscriber> subPtr);
        void registerPublisher(std::shared_ptr<Publisher> pubPtr);
        void registerService(std::shared_ptr<Service> srvPtr);
        void deregisterSubscriber(std::shared_ptr<Subscriber> subPtr);
        void deregisterPublisher(std::shared_ptr<Publisher> pubPtr);
        void deregisterService(std::shared_ptr<Service> srvPtr);

        bool requestServiceAddress(const std::string &service, URI &serviceURI, const std::string &inMsgType, const std::string &outMsgType);
        std::string callService(const URI &serviceURI, const std::string &request);

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
        std::map<std::string, std::set<std::shared_ptr<Service>>> services;       // Map of service names to Service objects

        std::mutex clientMutex;
        UDPClient client; // For talking to the Mediator
    };

}