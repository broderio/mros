#include "mros/core/subscriber.hpp"

namespace mros
{
    Subscriber::~Subscriber()
    {
        shutdown();
        for (auto &pub : pubs)
        {
            pub.second->close();
        }
        publicServer.close();
    }

    int Subscriber::getNumPublishers() const
    {
        return pubs.size();
    }

    std::string Subscriber::getTopic() const
    {
        return topic;
    }

    void Subscriber::shutdown()
    {
        if (shutdownFlag)
        {
            return;
        }
        
        shutdownFlag = true;
    }

    void Subscriber::runOnce()
    {
        int status;
        std::string inMsg, outMsg;

        // Send any outgoing requests
        if (outgoingRequests.size() > 0)
        {
            URI uri = outgoingRequests.front();
            outgoingRequests.pop();

            private_msgs::Request request;
            request.topic = topic;
            request.protocol = "TCP";

            URI subURI;
            publicServer.getURI(subURI);
            request.uri.hostname = subURI.ip;
            request.uri.port = subURI.port;

            outMsg = Parser::encode(request, CORE_TOPICS::SUB_REQUEST);

            status = publicServer.sendTo(outMsg, uri);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                Console::log(LogLevel::ERROR, "Failed to send Request message to Publisher at " + uri.toString());
            }
            else
            {
                awaitingResponses.insert(uri);
            }
        }

        // Read any incoming responses
        if (awaitingResponses.size() > 0)
        {
            URI sender;
            status = publicServer.receiveFrom(inMsg, MAX_MSG_SIZE, sender);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                Console::log(LogLevel::ERROR, "Failed to receive Response message from Publisher at " + sender.toString());
            }
            else if (inMsg.size() > 0 && awaitingResponses.find(sender) != awaitingResponses.end())
            {
                awaitingResponses.erase(sender);

                // Parse Response message
                private_msgs::Response response;
                if (!Parser::decode(inMsg, response))
                {
                    Console::log(LogLevel::ERROR, "Failed to decode Response message from Publisher at " + sender.toString());
                }
                else if (response.topic.data == topic && response.error.data.size() == 0)
                {
                    if (response.protocol.data != "TCP")
                    {
                        Console::log(LogLevel::WARN, "Unsupported protocol in Response message from Publisher at " + sender.toString());
                    }
                    else
                    {
                        URI pubURI = response.uri.toURI();
                        awaitingConnect.push(std::make_pair(sender, std::make_shared<TCPClient>(pubURI)));
                    }
                }
            }
        }

        // Connect to any Publishers
        if (awaitingConnect.size() > 0)
        {

            std::pair<URI, std::shared_ptr<TCPClient>> pair = awaitingConnect.front();
            awaitingConnect.pop();

            if (pair.second->isConnected()) 
            {
                private_msgs::URIStamped uri;
                URI subURI;
                publicServer.getURI(subURI);
                uri.uri.hostname = subURI.ip;
                uri.uri.port = subURI.port;

                outMsg = Parser::encode(uri, CORE_TOPICS::SUB_RESPONSE);
                pair.second->send(outMsg);
                pubs.insert(pair);
            }
            else {
                status = pair.second->connect();
                if (SOCKET_STATUS_IS_ERROR(status))
                {
                    Console::log(LogLevel::ERROR, "Failed to connect to Publisher at " + pair.first.toString() + " with status " + std::to_string(status));
                }
                else if (status == SOCKET_OP_IN_PROGRESS) {
                    awaitingConnect.push(pair);
                }
                else if (SOCKET_STATUS_IS_OK(status))
                {
                    private_msgs::URIStamped uri;
                    URI subURI;
                    publicServer.getURI(subURI);
                    uri.uri.hostname = subURI.ip;
                    uri.uri.port = subURI.port;

                    outMsg = Parser::encode(uri, CORE_TOPICS::SUB_RESPONSE);
                    status = pair.second->send(outMsg);
                    if (SOCKET_STATUS_IS_ERROR(status))
                    {
                        Console::log(LogLevel::ERROR, "Failed to send Response message to Publisher at " + pair.first.toString());
                    }
                    pubs.insert(pair);
                }
            }
        }

        // Read from any connected Publishers
        for (auto &pub : pubs)
        {
            status = pub.second->receive(inMsg, MAX_MSG_SIZE);
            if (SOCKET_STATUS_IS_ERROR(status))
            {
                Console::log(LogLevel::ERROR, "Failed to receive message from Publisher at " + pub.first.toString());
            }
            else if (inMsg.size() > 0)
            {
                if (msgQueue.size() == queueSize)
                {
                    msgQueue.pop();
                }
                msgQueue.push(inMsg);
            }
        }

        // Invoke the callback for each message in the queue
        if (msgQueue.size() > 0)
        {
            callbackHelper->invokeCallback(msgQueue.front());
            msgQueue.pop();
        }
    }
}