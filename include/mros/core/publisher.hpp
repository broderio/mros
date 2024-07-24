#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <thread>
#include <chrono>
#include <queue>
#include <mutex>

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
#include "mros/private_msgs/response.hpp"
#include "mros/private_msgs/disconnect.hpp"
#include "mros/private_msgs/uri.hpp"

#include "mros/common.hpp"
#include "mros/utils/console.hpp"

namespace mros
{

    class Node;

    class Publisher
    {

        friend class Node;

    public:
        ~Publisher();

        template <typename MsgType>
        void publish(const MsgType &msg)
        {
            static_assert(std::is_base_of<IMessage, MsgType>::value, "MsgType must inherit from IMessage");
            if (typeid(MsgType).name() != msgType)
            {
                throw std::runtime_error("MsgType must match the type of the Publisher");
            }
            
            std::lock_guard<std::mutex> lock(msgQueueMutex);
            if (msgQueue.size() >= queueSize)
            {
                msgQueue.pop();
            }

            std::string msgStr = Parser::encode(msg, 0);
            msgQueue.push(msgStr);
        }

        int getNumSubscribers() const;

        std::string getTopic() const;

        void shutdown();

    private:
        
        Publisher(const std::string &topic, const size_t &queueSize, const std::string &msgType);

        static std::shared_ptr<Publisher> create(const std::string &topic, const size_t &queueSize, const std::string &msgType);

        void runOnce();

        bool shutdownFlag;

        std::string topic;
        size_t queueSize;
        std::string msgType;

        std::mutex msgQueueMutex;
        std::queue<std::string> msgQueue;

        std::set<URI> awaitingRequests;                         // Populated by Node upon receiving a PUB_NOTIFY message from the mediator
        std::queue<URI> outgoingResponses;                      // Populated by Publisher upon receiving a SUB_REQUEST message from a Subscriber
        std::set<URI> awaitingAccept; // Populated by Publisher upon sending a PUB_RESPONSE message to a Subscriber
        std::queue<std::shared_ptr<TCPConnection>> awaitingURI;                // Populated by Publisher upon receiving a SUB_RESPONSE message from a Subscriber
        std::map<URI, std::shared_ptr<TCPConnection>> subs;                      // Populated by Publisher upon accepting a connection from a Subscriber

        UDPServer publicServer;  // For communicating with Subscribers
        TCPServer privateServer; // For accepting connections from Subscribers
    };

}