#include "mros/core/mediator.hpp"

namespace mros
{

    Mediator &Mediator::getInstance()
    {
        static Mediator instance;
        return instance;
    }

    void Mediator::init()
    {
        SignalHandler::init();
        Console::init("mediator");
        server.bind();

        std::string uri = getLocalIP() + ":" + std::to_string(MEDIATOR_PORT_NUM);
        Console::log(LogLevel::INFO, "Mediator hosting on URI: " + uri);
    }

    void Mediator::shutdown()
    {
        std_msgs::String msg;
        msg = "Mediator shutting down";
        std::string outMsg = Parser::encode(msg, CORE_TOPICS::MED_TERMINATE);

        for (auto &topic : topicMap)
        {
            for (const URIPair &publisher : topic.second.pubs)
            {
                server.sendTo(outMsg, publisher.clientURI);
            }
            for (const URIPair &subscriber : topic.second.subs)
            {
                server.sendTo(outMsg, subscriber.clientURI);
            }
        }

        for (auto &service : serviceMap)
        {
            server.sendTo(outMsg, service.second.srv.clientURI);
        }
    }

    void Mediator::spin()
    {
        std::thread t(&Mediator::run, this);
        t.join();
    }

    Mediator::Mediator()
    : server(URI("0.0.0.0", MEDIATOR_PORT_NUM)), shutdownFlag(false)
    {
    }

    void Mediator::run()
    {
        spinning = true;
        while (ok())
        {
            std::string inMsg;
            URI clientURI;
            // std::cout << "Waiting for message..." << std::endl;
            int status = server.receiveFrom(inMsg, 1024, clientURI);

            // If there was an error
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                Console::log(LogLevel::ERROR, "UDPServer::receiveFrom failed.");
                return;
            }
            // If the operation would block
            else if (status == SOCKET_OP_WOULD_BLOCK)
            {
                continue;
            }
            // If the message was a PING or PONG
            else if (inMsg.size() == 1)
            {
                if ((uint8_t)inMsg[0] == PING_MSG)
                {
                    status = server.sendTo(std::string(1, PONG_MSG), clientURI);
                    if (SOCKET_STATUS_IS_ERROR(status))
                    {
                        Console::log(LogLevel::ERROR, "UDPServer::sendTo failed.");
                    }
                    continue;
                }
                else if ((uint8_t)inMsg[0] == PONG_MSG)
                {
                    Console::log(LogLevel::INFO, "Received PONG from " + clientURI.toString());
                    continue;
                }
            }

            Header header;
            std::string msg;
            uint16_t topicId;
            Parser::getTopicID(inMsg, topicId);

            private_msgs::Register registerMsg;
            std_msgs::String serviceRequest;
            switch (topicId)
            {
            case CORE_TOPICS::PUB_REGISTER:
                Parser::decode(inMsg, registerMsg);
                registerPublisher(registerMsg, clientURI);
                break;
            case CORE_TOPICS::SUB_REGISTER:
                Parser::decode(inMsg, registerMsg);
                registerSubscriber(registerMsg, clientURI);
                break;
            case CORE_TOPICS::SRV_REGISTER:
                Parser::decode(inMsg, registerMsg);
                registerService(registerMsg, clientURI);
                break;
            case CORE_TOPICS::PUB_DEREGISTER:
                Parser::decode(inMsg, registerMsg);
                deregisterPublisher(registerMsg, clientURI);
                break;
            case CORE_TOPICS::SUB_DEREGISTER:
                Parser::decode(inMsg, registerMsg);
                deregisterSubscriber(registerMsg, clientURI);
                break;
            case CORE_TOPICS::SRV_DEREGISTER:
                Parser::decode(inMsg, registerMsg);
                deregisterService(registerMsg, clientURI);
                break;
            case CORE_TOPICS::SRV_REQUEST:
                Parser::decode(inMsg, serviceRequest);
                handleServiceRequest(serviceRequest, clientURI);
                break;
            default:
                Console::log(LogLevel::ERROR, "Unknown topic ID: " + std::to_string(topicId));
                return;
            }
        }
        Console::log(LogLevel::INFO, "Mediator shutting down");
        shutdown();
    }

    bool Mediator::ok()
    {
        return SignalHandler::ok() && !shutdownFlag;
    }

    void Mediator::registerPublisher(const private_msgs::Register &msg, const URI &clientURI)
    {
        std::string topic = msg.topic;
        std::string msgType = msg.msgType;
        URI uri = msg.uri.toURI(); // URI that will be sent to existing subscribers

        std::vector<private_msgs::URI> subURIs; // URIs that will be sent to newly registered publisher
        std::string error = "";

        URIPair uriPair = {clientURI, uri};

        // Check if topic already exists
        if (topic.empty())
        {
            error = "Topic is an empty string";
            Console::log(LogLevel::ERROR, "Received empty topic string in Register message from Publisher " + uri.toString());
        }
        else if (topicMap.find(topic) == topicMap.end())
        {
            // Topic does not exist, create new topic
            topicMap[topic] = Topic();
            topicMap[topic].msgType = msgType;
        }
        else
        {
            // Topic already exists, check if publisher already exists
            if (topicMap[topic].msgType != msgType)
            {
                // Topic exists with different message type
                error = "Topic \"" + topic + "\" already exists with different message type " + topicMap[topic].msgType;
                Console::log(LogLevel::ERROR, error);
            }
            else if (topicMap[topic].pubs.find(uriPair) != topicMap[topic].pubs.end())
            {
                // Publisher already exists
                error = "Publisher " + uri.toString() + " already exists for topic " + topic;
                Console::log(LogLevel::ERROR, error);
            }
        }

        // If no errors, add publisher to topic
        if (error.size() == 0)
        {
            // Add publisher to topic
            topicMap[topic].pubs.insert(uriPair);
            Console::log(LogLevel::INFO, "Registered publisher " + uriPair.serverURI.toString() + " from node " + uriPair.clientURI.toString() + " on topic \"" + topic + "\"");

            // Add existing subscribers to response
            for (const URIPair &subscriber : topicMap[topic].subs)
            {
                subURIs.push_back(private_msgs::URI(subscriber.serverURI.ip, subscriber.serverURI.port));
            }
        }

        private_msgs::Notify response;
        response.header.stamp = Time::getTimeNow();
        response.error = error;
        response.topic = String(topic);
        response.uris = subURIs;

        // Send response to publisher
        std::string outMsg = Parser::encode(response, CORE_TOPICS::PUB_NOTIFY);
        int status = server.sendTo(outMsg, uriPair.clientURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "UDPServer::sendTo failed.");
        }

        // Send response to all subscribers of topic if no errors
        if (error.size() == 0 && topicMap[topic].subs.size() > 0)
        {
            response.uris.clear();
            response.uris.push_back(private_msgs::URI(uri.ip, uri.port));
            outMsg = Parser::encode(response, CORE_TOPICS::SUB_NOTIFY);
            for (const URIPair &subscriber : topicMap[topic].subs)
            {
                status = server.sendTo(outMsg, subscriber.clientURI);
                if (SOCKET_STATUS_IS_ERROR(status))
                {
                    Console::log(LogLevel::ERROR, "UDPServer::sendTo failed.");
                }
            }
        }
    }

    void Mediator::registerSubscriber(const private_msgs::Register &msg, const URI &clientURI)
    {
        std::string topic = msg.topic;
        std::string msgType = msg.msgType;
        URI uri = msg.uri.toURI(); // URI that will be sent to existing publishers

        std::vector<private_msgs::URI> pubURIs; // URIs that will be sent to newly registered subscriber
        std::string error = "";

        URIPair uriPair = {clientURI, uri};

        // Check if topic already exists
        if (topic.empty())
        {
            error = "Topic is an empty string";
            Console::log(LogLevel::ERROR, "Received empty topic string in Register message from Subscriber " + uri.toString());
        }
        else if (topicMap.find(topic) == topicMap.end())
        {
            // Topic does not exist, create new topic
            topicMap[topic] = Topic();
            topicMap[topic].msgType = msgType;
        }
        else
        {
            // Topic already exists, check if subscriber already exists
            if (topicMap[topic].msgType != msgType)
            {
                // Topic exists with different message type
                error = "Topic \"" + topic + "\" already exists with different message type \"" + topicMap[topic].msgType + "\"";
                Console::log(LogLevel::ERROR, error);
            }
            else if (topicMap[topic].subs.find(uriPair) != topicMap[topic].subs.end())
            {
                // Subscriber already exists
                error = "Subscriber " + uri.toString() + " already exists for topic \"" + topic + "\"";
                Console::log(LogLevel::ERROR, error);
            }
        }

        // If no errors, add subscriber to topic
        if (error.size() == 0)
        {
            // Add subscriber to topic
            topicMap[topic].subs.insert(uriPair);
            Console::log(LogLevel::INFO, "Registered subscriber " + uriPair.serverURI.toString() + " from node " + uriPair.clientURI.toString() + " on topic \"" + topic + "\"");

            // Add existing publishers to response
            for (const URIPair &publisher : topicMap[topic].pubs)
            {
                pubURIs.push_back(private_msgs::URI(publisher.serverURI.ip, publisher.serverURI.port));
            }
        }

        private_msgs::Notify response;
        response.header.stamp = Time::getTimeNow();
        response.error = error;
        response.topic = String(topic);
        response.uris = pubURIs;

        // Send response to subscriber
        std::string outMsg = Parser::encode(response, CORE_TOPICS::SUB_NOTIFY);
        int status = server.sendTo(outMsg, uriPair.clientURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "UDPServer::sendTo failed.");
        }

        // Send response to all publishers of topic if no errors
        if (error.size() == 0 && topicMap[topic].pubs.size() > 0)
        {
            response.uris.clear();
            response.uris.push_back(private_msgs::URI(uri.ip, uri.port));
            outMsg = Parser::encode(response, CORE_TOPICS::PUB_NOTIFY);
            for (const URIPair &publisher : topicMap[topic].pubs)
            {
                status = server.sendTo(outMsg, publisher.clientURI);
                if (SOCKET_STATUS_IS_ERROR(status))
                {
                    Console::log(LogLevel::ERROR, "UDPServer::sendTo failed.");
                }
            }
        }
    }

    void Mediator::registerService(const private_msgs::Register &msg, const URI &clientURI)
    {
        std::string service = msg.topic;
        std::string msgTypes = msg.msgType;
        std::string inMsgType = msgTypes.substr(0, msgTypes.find(','));
        std::string outMsgType = msgTypes.substr(msgTypes.find(',') + 1);

        URI uri = msg.uri.toURI(); // URI that will be used in the remote procedure call

        std::string error = "";

        if (service.empty())
        {
            error = "Service is an empty string";
            Console::log(LogLevel::ERROR, "Received empty service string in Register message from Service " + uri.toString());
        }
        else if (serviceMap.find(service) == serviceMap.end())
        {
            // Service does not exist, create new service
            serviceMap[service] = Service();
            serviceMap[service].srv.clientURI = clientURI;
            serviceMap[service].srv.serverURI = uri;
            serviceMap[service].inMsgType = inMsgType;
            serviceMap[service].outMsgType = outMsgType;
            Console::log(LogLevel::INFO, "Registered service \"" + service + "\" from node " + clientURI.toString());
        }
        else
        {
            // Service already exists
            error = "Service \"" + service + "\" already exists";
            Console::log(LogLevel::ERROR, error);
        }

        private_msgs::Notify response;
        response.header.stamp = Time::getTimeNow();
        response.error = error;
        response.topic = service;

        // Send response to service
        std::string outMsg = Parser::encode(response, CORE_TOPICS::SRV_NOTIFY);
        int status = server.sendTo(outMsg, clientURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "UDPServer::sendTo failed.");
        }
    }

    void Mediator::deregisterPublisher(const private_msgs::Register &msg, const URI &clientURI)
    {
        std::string topic = msg.topic;
        std::string msgType = msg.msgType;
        URI uri = msg.uri.toURI(); // URI that will be deleted from map and sent to subscribers

        URIPair uriPair = {clientURI, uri};

        // Check if topic exists
        if (topicMap.find(topic) == topicMap.end())
        {
            Console::log(LogLevel::ERROR, "Topic \"" + topic + "\" from Deregister message from Publisher " + uriPair.clientURI.toString() + " not found.");
            return;
        }

        // Check if publisher exists
        if (topicMap[topic].pubs.find(uriPair) == topicMap[topic].pubs.end())
        {
            Console::log(LogLevel::ERROR, "Publisher " + uriPair.serverURI.toString() + " from node " + uriPair.clientURI.toString() + " not found.");
            return;
        }

        // Erase publisher from topic
        topicMap[topic].pubs.erase(uriPair);
        Console::log(LogLevel::INFO, "Deregistered publisher " + uriPair.serverURI.toString() + " from node " + uriPair.clientURI.toString() + " on topic \"" + topic + "\"");

        private_msgs::Disconnect response;
        response.header.stamp = Time::getTimeNow();
        response.uri = msg.uri;
        response.topic = msg.topic;

        // Send response to all subscribers of topic
        int status;
        std::string outMsg = Parser::encode(response, CORE_TOPICS::SUB_DISCONNECT);
        for (const URIPair &subscriber : topicMap[topic].subs)
        {
            status = server.sendTo(outMsg, subscriber.clientURI);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                Console::log(LogLevel::ERROR, "UDPServer::sendTo failed.");
            }
        }
    }

    void Mediator::deregisterSubscriber(const private_msgs::Register &msg, const URI &clientURI)
    {
        std::string topic = msg.topic;
        std::string msgType = msg.msgType;
        URI uri = msg.uri.toURI(); // URI that will be deleted from map and sent to publishers

        URIPair uriPair = {clientURI, uri};

        // Check if topic exists
        if (topicMap.find(topic) == topicMap.end())
        {
            Console::log(LogLevel::ERROR, "Topic \"" + topic + "\" from Deregister message from Subscriber " + uriPair.clientURI.toString() + " not found.");
            return;
        }

        // Check if subscriber exists
        if (topicMap[topic].subs.find(uriPair) == topicMap[topic].subs.end())
        {
            Console::log(LogLevel::ERROR, "Subscriber " + uriPair.serverURI.toString() + " from node " + uriPair.clientURI.toString() + " not found.");
            return;
        }

        // Erase subscriber from topic
        topicMap[topic].subs.erase(uriPair);
        Console::log(LogLevel::INFO, "Deregistered subscriber " + uriPair.serverURI.toString() + " from node " + uriPair.clientURI.toString() + " on topic \"" + topic + "\"");

        private_msgs::Disconnect response;
        response.header.stamp = Time::getTimeNow();
        response.uri = msg.uri;
        response.topic = msg.topic;

        // Send response to all publishers of topic
        int status;
        std::string outMsg = Parser::encode(response, CORE_TOPICS::PUB_DISCONNECT);
        for (const URIPair &publisher : topicMap[topic].pubs)
        {
            status = server.sendTo(outMsg, publisher.clientURI);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                Console::log(LogLevel::ERROR, "UDPServer::sendTo failed.");
            }
        }
    }

    void Mediator::deregisterService(const private_msgs::Register &msg, const URI &clientURI)
    {
        std::string service = msg.topic;
        std::string msgTypes = msg.msgType;
        std::string inMsgType = msgTypes.substr(0, msgTypes.find(','));
        std::string outMsgType = msgTypes.substr(msgTypes.find(',') + 1);

        URI uri = msg.uri.toURI(); // URI that will be deleted from map

        // Check if service exists
        if (serviceMap.find(service) == serviceMap.end())
        {
            Console::log(LogLevel::ERROR, "Service \"" + service + "\" from Deregister message from Service " + uri.toString() + " not found.");
            return;
        }

        // Check if service URI matches
        if (serviceMap[service].srv.serverURI != uri || serviceMap[service].srv.clientURI != clientURI)
        {
            Console::log(LogLevel::ERROR, "Service URI " + uri.toString() + " does not match registered URI " + serviceMap[service].srv.serverURI.toString());
            return;
        }

        // Erase service from map
        serviceMap.erase(service);
        Console::log(LogLevel::INFO, "Deregistered service " + uri.toString() + " from node " + clientURI.toString());
    }

    void Mediator::handleServiceRequest(const std_msgs::String &msg, const URI &clientURI)
    {
        std::string service = msg;
        std::string error = "";

        // Check if service exists
        if (serviceMap.find(service) == serviceMap.end())
        {
            error = "Service \"" + service + "\" not found";
            Console::log(LogLevel::WARN, error);
        }

        private_msgs::Response response;
        if (error.size() == 0)
        {
            response.header.stamp = Time::getTimeNow();
            response.topic = service;
            response.protocol = serviceMap[service].inMsgType + "," + serviceMap[service].outMsgType;

            URI serviceURI = serviceMap[service].srv.serverURI;
            response.uri = private_msgs::URI(serviceURI.ip, serviceURI.port);
        }
        else
        {
            response.error = String(error);
        }

        std::string outMsg = Parser::encode(response, CORE_TOPICS::SRV_REQUEST);
        int status = server.sendTo(outMsg, clientURI);
        if (SOCKET_STATUS_IS_ERROR(status))
        {
            Console::log(LogLevel::ERROR, "UDPServer::sendTo failed.");
        }
    }

    bool Mediator::URIPair::operator<(const URIPair &other) const
    {
        return clientURI < other.clientURI || (clientURI == other.clientURI && serverURI < other.serverURI);
    }

} // namespace mros