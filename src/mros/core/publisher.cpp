#include "mros/core/publisher.hpp"

namespace mros
{
    Publisher::~Publisher()
    {
        shutdown();
        for (auto &sub : subs)
        {
            sub.second->close();
        }
        publicServer.close();
        privateServer.close();
    }

    int Publisher::getNumSubscribers() const
    {
        return subs.size();
    }

    std::string Publisher::getTopic() const
    {
        return topic;
    }

    void Publisher::shutdown()
    {
        if (shutdownFlag)
        {
            return;
        }
        shutdownFlag = true;
    }

    void Publisher::runOnce()
    {
        int status;
        std::string inMsg;
        std::string outMsg;

        // Check for a new Subscriber Request
        if (awaitingRequests.size() > 0)
        {
            URI sender;
            status = publicServer.receiveFrom(inMsg, MAX_MSG_SIZE, sender);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                Console::log(LogLevel::ERROR, "UDPServer::receiveFrom() failed");
            }
            else if (inMsg.size() > 0 && awaitingRequests.find(sender) != awaitingRequests.end())
            {
                awaitingRequests.erase(sender);

                // Parse the Request message
                private_msgs::Request request;
                if (!Parser::decode(inMsg, request))
                {
                    Console::log(LogLevel::ERROR, "Failed to decode Request message from Subscriber");
                }
                else if (request.topic.data == topic)
                {
                    if (request.protocol.data != "TCP")
                    {
                        Console::log(LogLevel::WARN, "Unsupported protocol in Request message from Subscriber at " + sender.toString());
                    }
                    else
                    {
                        outgoingResponses.push(sender);
                    }
                }
            }
        }

        // Send any outgoing responses
        if (outgoingResponses.size() > 0)
        {
            URI uri = outgoingResponses.front();
            outgoingResponses.pop();

            private_msgs::Response response;
            response.topic.data = topic;
            response.protocol.data = "TCP";
            response.error = "";

            URI pubURI;
            privateServer.getURI(pubURI);
            response.uri.hostname = pubURI.ip;
            response.uri.port = pubURI.port;

            outMsg = Parser::encode(response, CORE_TOPICS::PUB_RESPONSE);

            status = publicServer.sendTo(outMsg, uri);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                Console::log(LogLevel::ERROR, "Failed to send Response message to Subscriber at " + uri.toString());
                outgoingResponses.push(uri);
            }
            else
            {
                awaitingAccept.insert(uri);
            }
        }

        // Accept any incoming connections
        if (awaitingAccept.size() > 0)
        {
            std::shared_ptr<TCPConnection> connection;
            status = privateServer.accept(connection);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                Console::log(LogLevel::ERROR, "TCPServer::accept() failed");
            }
            else if (SOCKET_STATUS_IS_OK(status))
            {
                URI clientURI;
                connection->getClientURI(clientURI);
                awaitingURI.push(connection);
            }
        }

        if (awaitingURI.size() > 0)
        {
            std::shared_ptr<TCPConnection> connection = awaitingURI.front();
            awaitingURI.pop();

            status = connection->receive(inMsg, MAX_MSG_SIZE);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                Console::log(LogLevel::ERROR, "TCPConnection::receive() failed");
            }
            else if (status == SOCKET_OP_WOULD_BLOCK)
            {
                awaitingURI.push(connection);
            }
            else
            {
                private_msgs::URIStamped uriMsg;
                if (!Parser::decode(inMsg, uriMsg))
                {
                    Console::log(LogLevel::ERROR, "Failed to decode URI message from Subscriber");
                }
                else
                {
                    URI uri = uriMsg.uri.toURI();
                    if (awaitingAccept.find(uri) != awaitingAccept.end())
                    {
                        subs[uri] = connection;
                        awaitingAccept.erase(uri);
                    }
                }
            }
        }

        // Send any outgoing messages
        if (msgQueue.size() > 0)
        {
            {
                std::lock_guard<std::mutex> lock(msgQueueMutex);
                outMsg = msgQueue.front();
                msgQueue.pop();
            }

            for (auto &sub : subs)
            {
                status = sub.second->send(outMsg);
                if (SOCKET_STATUS_IS_ERROR(status))
                {
                    Console::log(LogLevel::ERROR, "Failed to send message to Subscriber at " + sub.first.toString());
                }
            }
        }
    }

    Publisher::Publisher(const std::string &topic, const size_t &queueSize, const std::string &msgType)
        : shutdownFlag(false), topic(topic), queueSize(queueSize), msgType(msgType), msgQueue(), publicServer(URI(getLocalIP(), 0)), privateServer(URI(getLocalIP(), 0))
    {
        publicServer.bind();
        privateServer.bind();
        privateServer.listen();
    }

    std::shared_ptr<Publisher> Publisher::create(const std::string &topic, const size_t &queueSize, const std::string &msgType)
    {
        std::shared_ptr<Publisher> pubPtr(new Publisher(topic, queueSize, msgType));
        return pubPtr;
    }
}