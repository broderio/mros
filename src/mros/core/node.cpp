#include "mros/core/node.hpp"

namespace mros
{

    Node &Node::getInstance()
    {
        static Node instance;
        return instance;
    }

    Node::~Node()
    {
        if (!shutdownFlag)
        {
            shutdown();
        }
        
        if (spinThreadID != std::this_thread::get_id() && spinThread.joinable())
        {
            spinThread.join();
        }
    }

    void Node::init(const std::string &name, const URI &coreURI)
    {
        if (initialized)
        {
            Console::log(LogLevel::WARN, "Node has already been initialized");
            return;
        }

        this->name = name;
        this->coreURI = coreURI;
        this->spinning = false;
        this->shutdownFlag = false;

        Console::init(name);
        SignalHandler::init();

        std::string inMsg;
        URI sender;
        int status;
        int attempts = 0;
        while (inMsg.size() == 0)
        {
            status = client.sendTo(std::string(1, PING_MSG), coreURI);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                Console::log(LogLevel::ERROR, "UDPClient::sendTo() failed");
                goto error;
            }

            sleep(100);

            status = client.receiveFrom(inMsg, MAX_MSG_SIZE, sender);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                Console::log(LogLevel::ERROR, "UDPClient::receiveFrom() failed");
                goto error;
            }

            if (attempts > 10)
            {
            error:
                Console::log(LogLevel::ERROR, "Failed to connect to Mediator");
                throw std::runtime_error("Failed to connect to Mediator");
            }
            attempts++;
        }
        this->initialized = true;
    }

    void Node::spin(bool block)
    {
        if (!initialized)
        {
            Console::log(LogLevel::ERROR, "Node has not been initialized");
            throw std::runtime_error("Node has not been initialized");
            return;
        }

        if (spinning)
        {
            Console::log(LogLevel::WARN, "Node is already spinning");
            return;
        }

        spinThread = std::thread(&Node::run, this);
        spinning = true;
        if (block)
        {
            spinThread.join();
        }
    }

    void Node::spinOnce()
    {
        if (!initialized)
        {
            Console::log(LogLevel::ERROR, "Node has not been initialized");
            throw std::runtime_error("Node has not been initialized");
            return;
        }

        if (spinning)
        {
            Console::log(LogLevel::WARN, "Node is already spinning");
            return;
        }
        runOnce();
    }

    void Node::shutdown()
    {
        if (!initialized)
        {
            Console::log(LogLevel::ERROR, "Node has not been initialized");
            throw std::runtime_error("Node has not been initialized");
            return;
        }

        if (shutdownFlag)
        {
            Console::log(LogLevel::WARN, "Node is already shutting down");
            return;
        }

        shutdownFlag = true;

        for (auto &topic : publishers)
        {
            for (auto &pub : topic.second)
            {
                deregisterPublisher(pub);
            }
        }
        publishers.clear();

        for (auto &topic : subscribers)
        {
            for (auto &sub : topic.second)
            {
                deregisterSubscriber(sub);
            }
        }
        subscribers.clear();

        for (auto &srv : services)
        {
            for (auto &service : srv.second)
            {
                deregisterService(service);
            }
        }
        services.clear();

        client.close();
    }

    bool Node::ok()
    {
        if (!initialized)
        {
            Console::log(LogLevel::ERROR, "Node has not been initialized");
            throw std::runtime_error("Node has not been initialized");
        }

        return SignalHandler::ok() && !shutdownFlag;
    }

    Node::Node()
    {
        initialized = false;
    }

    void Node::run()
    {
        spinThreadID = std::this_thread::get_id();
        while (ok())
        {
            try
            {
                runOnce();
            }
            catch (const std::exception &e)
            {
                Console::log(LogLevel::ERROR, "Error thrown in Node::run(): " + std::string(e.what()));
                return;
            }
        }
    }

    void Node::runOnce()
    {
        // Check if any messages have been sent to our UDP server
        std::string inMsg;
        URI sender;
        int status = client.receiveFrom(inMsg, MAX_MSG_SIZE, sender);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "Error receiving message from Mediator");
        }

        // If we have a message, parse it
        if (inMsg.size() == 1)
        {
            if ((uint8_t)inMsg[0] == PING_MSG)
            {
                client.sendTo(std::string(1, PONG_MSG), sender);
            }
            else if ((uint8_t)inMsg[0] == PONG_MSG)
            {
                Console::log(LogLevel::DEBUG, "Received PONG from Mediator");
            }
        }
        else if (inMsg.size() > 0)
        {
            uint16_t topicId;
            // Parse message based on topic field
            if (Parser::getTopicID(inMsg, topicId))
            {
                private_msgs::Notify notify;
                private_msgs::Disconnect disconnect;
                if (topicId == CORE_TOPICS::PUB_NOTIFY || topicId == CORE_TOPICS::SUB_NOTIFY)
                {
                    if (!Parser::decode(inMsg, notify))
                    {
                        Console::log(LogLevel::ERROR, "Could not decode message");
                        return;
                    }
                }
                else if (topicId == CORE_TOPICS::PUB_DISCONNECT || topicId == CORE_TOPICS::SUB_DISCONNECT)
                {
                    if (!Parser::decode(inMsg, disconnect))
                    {
                        Console::log(LogLevel::ERROR, "Could not decode message");
                        return;
                    }
                }
                else if (topicId == CORE_TOPICS::MED_TERMINATE)
                {
                    Console::log(LogLevel::INFO, "Received termination message from Mediator");
                    shutdown();
                    return;
                }

                switch (topicId)
                {
                case CORE_TOPICS::PUB_NOTIFY:
                    handlePubNotify(notify); // Queues up subscribers to be connected to
                    break;
                case CORE_TOPICS::SUB_NOTIFY:
                    handleSubNotify(notify); // Queues up publishers to be connected to
                    break;
                case CORE_TOPICS::PUB_DISCONNECT:
                    handlePubDisconnect(disconnect);
                    break;
                case CORE_TOPICS::SUB_DISCONNECT:
                    handleSubDisconnect(disconnect);
                    break;
                }
            }
        }

        // Run each publisher, subscriber, and service
        std::vector<std::shared_ptr<Publisher>> toDeletePub;
        for (auto &topic : publishers)
        {
            for (auto &pub : topic.second)
            {
                if (pub->shutdownFlag)
                {
                    deregisterPublisher(pub);
                    toDeletePub.push_back(pub);
                    continue;
                }
                pub->runOnce();
            }
        }
        for (auto &pub : toDeletePub)
        {
            publishers[pub->getTopic()].erase(pub);
        }

        std::vector<std::shared_ptr<Subscriber>> toDeleteSub;
        for (auto &topic : subscribers)
        {
            for (auto &sub : topic.second)
            {
                if (sub->shutdownFlag)
                {
                    deregisterSubscriber(sub);
                    toDeleteSub.push_back(sub);
                    continue;
                }
                sub->runOnce();
            }
        }
        for (auto &sub : toDeleteSub)
        {
            subscribers[sub->getTopic()].erase(sub);
        }

        std::vector<std::shared_ptr<Service>> toDeleteSrv;
        for (auto &srv : services)
        {
            for (auto &service : srv.second)
            {
                if (service->shutdownFlag)
                {
                    deregisterService(service);
                    toDeleteSrv.push_back(service);
                    continue;
                }
                service->runOnce();
            }
        }
        for (auto &service : toDeleteSrv)
        {
            services[service->getService()].erase(service);
        }
    }

    void Node::registerPublisher(std::shared_ptr<Publisher> pub)
    {
        private_msgs::Register msg;
        msg.header.stamp = Time::getTimeNow();
        msg.header.frame_id = name;
        msg.topic = pub->topic;
        msg.msgType = pub->msgType;

        URI uri;
        pub->publicServer.getURI(uri);
        msg.uri = private_msgs::URI(uri.ip, uri.port);

        int status = client.sendTo(Parser::encode(msg, CORE_TOPICS::PUB_REGISTER), coreURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "Failed to send message to Mediator");
        }
    }

    void Node::registerSubscriber(std::shared_ptr<Subscriber> sub)
    {
        private_msgs::Register msg;
        msg.header.stamp = Time::getTimeNow();
        msg.header.frame_id = name;
        msg.topic = sub->topic;
        msg.msgType = sub->msgType;

        URI uri;
        sub->publicServer.getURI(uri);
        msg.uri = private_msgs::URI(uri.ip, uri.port);

        int status = client.sendTo(Parser::encode(msg, CORE_TOPICS::SUB_REGISTER), coreURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "Failed to send message to Mediator");
        }
    }

    void Node::registerService(std::shared_ptr<Service> srv)
    {
        private_msgs::Register msg;
        msg.header.stamp = Time::getTimeNow();
        msg.header.frame_id = name;
        msg.topic = srv->service;
        msg.msgType = srv->inMsgType + "," + srv->outMsgType;

        URI uri;
        srv->publicServer.getURI(uri);
        msg.uri = private_msgs::URI(uri.ip, uri.port);

        int status = client.sendTo(Parser::encode(msg, CORE_TOPICS::SRV_REGISTER), coreURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "Failed to send message to Mediator");
        }
    }

    void Node::deregisterPublisher(std::shared_ptr<Publisher> pub)
    {
        private_msgs::Register msg;
        msg.header.stamp = Time::getTimeNow();
        msg.header.frame_id = name;
        msg.topic = pub->topic;
        msg.msgType = pub->msgType;

        URI uri;
        pub->publicServer.getURI(uri);
        msg.uri = private_msgs::URI(uri.ip, uri.port);

        int status = client.sendTo(Parser::encode(msg, CORE_TOPICS::PUB_DEREGISTER), coreURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "Failed to send message to Mediator");
        }
    }

    void Node::deregisterSubscriber(std::shared_ptr<Subscriber> sub)
    {
        private_msgs::Register msg;
        msg.header.stamp = Time::getTimeNow();
        msg.header.frame_id = name;
        msg.topic = sub->topic;
        msg.msgType = sub->msgType;

        URI uri;
        sub->publicServer.getURI(uri);
        msg.uri = private_msgs::URI(uri.ip, uri.port);

        int status = client.sendTo(Parser::encode(msg, CORE_TOPICS::SUB_DEREGISTER), coreURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "Failed to send message to Mediator");
        }
    }

    void Node::deregisterService(std::shared_ptr<Service> srv)
    {
        private_msgs::Register msg;
        msg.header.stamp = Time::getTimeNow();
        msg.header.frame_id = name;
        msg.topic = srv->service;
        msg.msgType = srv->inMsgType + "," + srv->outMsgType;

        URI uri;
        srv->publicServer.getURI(uri);
        msg.uri = private_msgs::URI(uri.ip, uri.port);

        int status = client.sendTo(Parser::encode(msg, CORE_TOPICS::SRV_DEREGISTER), coreURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "Failed to send message to Mediator");
        }
    }

    bool Node::requestServiceAddress(const std::string &service, URI &serviceURI, const std::string &inMsgType, const std::string &outMsgType)
    {
        UDPClient tmpClient(false);
        std_msgs::String msg = service;

        int status = tmpClient.sendTo(Parser::encode(msg, CORE_TOPICS::SRV_REQUEST), coreURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "Failed to send message to Mediator");
            return false;
        }

        std::string inMsg;
        URI sender;
        status = tmpClient.receiveFrom(inMsg, MAX_MSG_SIZE, sender);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "Failed to receive message from Mediator");
            return false;
        }

        private_msgs::Response response;
        if (!Parser::decode(inMsg, response))
        {
            Console::log(LogLevel::ERROR, "Could not decode message");
            return false;
        }

        if (response.error.data.size() > 0)
        {
            Console::log(LogLevel::WARN, response.error.data);
            return false;
        }

        std::string msgTypes = response.protocol.data;
        std::string inMsgTypeIn = msgTypes.substr(0, msgTypes.find(","));
        std::string outMsgTypeIn = msgTypes.substr(msgTypes.find(",") + 1);

        if (inMsgTypeIn != inMsgType || outMsgTypeIn != outMsgType)
        {
            Console::log(LogLevel::WARN, "Service message types do not match");
            return false;
        }

        serviceURI = response.uri.toURI();
        return true;
    }

    bool Node::callService(const URI &serviceURI, const std::string &request, std::string &response)
    {
        response = "";

        UDPClient tmpClient(false);
        int status = tmpClient.sendTo(request, serviceURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "Failed to send message to Service");
            return false;
        }

        std::string inMsg;
        URI sender;
        status = tmpClient.receiveFrom(inMsg, MAX_MSG_SIZE, sender);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "Failed to receive message from Service");
            return false;
        }

        response = inMsg;
        return true;
    }

    void Node::handlePubNotify(const private_msgs::Notify &notify)
    {
        std::string error = notify.error.data;
        if (error.size() > 0)
        {
            Console::log(LogLevel::ERROR, error);
            shutdown();
        }

        std::string topic = notify.topic.data;
        for (auto &pub : publishers[topic])
        {
            for (const private_msgs::URI &uri : notify.uris)
            {
                pub->awaitingRequests.insert(uri.toURI());
            }
        }
    }

    void Node::handleSubNotify(const private_msgs::Notify &notify)
    {
        std::string error = notify.error.data;
        if (error.size() > 0)
        {
            Console::log(LogLevel::ERROR, error);
            shutdown();
        }

        std::string topic = notify.topic.data;
        for (auto &sub : subscribers[topic])
        {
            for (const private_msgs::URI &uri : notify.uris)
            {
                sub->outgoingRequests.push(uri.toURI());
            }
        }
    }

    void Node::handlePubDisconnect(const private_msgs::Disconnect &disconnect)
    {
        for (auto &pub : publishers[disconnect.topic.data])
        {
            pub->subs.erase(disconnect.uri.toURI());
        }
    }

    void Node::handleSubDisconnect(const private_msgs::Disconnect &disconnect)
    {
        for (auto &sub : subscribers[disconnect.topic.data])
        {
            sub->pubs.erase(disconnect.uri.toURI());
        }
    }
}