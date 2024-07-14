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
#include "mros/private_msgs/response.hpp"
#include "mros/private_msgs/disconnect.hpp"
#include "mros/private_msgs/uri.hpp"

#include "mros/common.hpp"

namespace mros
{

    class Node;

    class CallbackHelperBase
    {
    public:
        virtual void invokeCallback(const std::string &msg) = 0;
    };

    template <typename MsgType>
    class CallbackHelper : public CallbackHelperBase
    {
    public:
        CallbackHelper(std::function<void(const MsgType &)> callback)
            : callback(callback) {}

        void invokeCallback(const std::string &msg) override
        {
            MsgType m;
            Parser::decode(msg, m);
            callback(m);
        }

    private:
        std::function<void(const MsgType &)> callback;
    };

    class Subscriber
    {

        friend class Node;

    public:
        ~Subscriber();

        int getNumPublishers() const;

        std::string getTopic() const;

        void shutdown();

    private:

        template <typename MsgType>
        Subscriber(const std::string &topic, const size_t &queueSize, std::function<void(const MsgType &)> callback)
        : publicServer(URI(getIPAddr(), 0)) {
            static_assert(std::is_base_of<IMessage, MsgType>::value, "MsgType must inherit from IMessage");
            msgType = typeid(MsgType).name();
            this->topic = topic;
            this->queueSize = queueSize;
            callbackHelper = std::make_shared<CallbackHelper<MsgType>>(callback);
            shutdownFlag = false;
            publicServer.bind();
        }

        template <typename MsgType>
        static std::shared_ptr<Subscriber> create(const std::string &topic, const size_t &queueSize, std::function<void(const MsgType &)> callback)
        {
            std::shared_ptr<Subscriber> subSharedPtr(new Subscriber(topic, queueSize, callback));
            return subSharedPtr;
        }

        void runOnce();

        std::shared_ptr<CallbackHelperBase> callbackHelper;

        bool shutdownFlag;

        std::string msgType;
        std::string topic;
        size_t queueSize;

        std::queue<std::string> msgQueue;

        std::queue<URI> outgoingRequests;                      // Populated by Node upon receiving a SUB_NOTIFY message from the mediator
        std::set<URI> awaitingResponses;                       // Populated by Subscriber upon sending a SUB_REQUEST message to a Publisher
        std::queue<std::pair<URI, std::shared_ptr<TCPClient>>> awaitingConnect; // Populated by Subscriber upon receiving a PUB_RESPONSE message from a Publisher
        std::map<URI, std::shared_ptr<TCPClient>> pubs;                         // Populated by Subscriber when a connection is established with a Publisher

        UDPServer publicServer;
    };

}