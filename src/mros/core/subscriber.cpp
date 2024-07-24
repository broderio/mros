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

        // Send any outgoing requests
        if (outgoingRequests.size() > 0)
        {
            URI uri = outgoingRequests.front();
            outgoingRequests.pop();

            private_msgs::Request request;
            request.topic = topic;
            request.protocol = "TCP";
            URI subURI = publicServer.getURI();
            request.uri.hostname = subURI.ip;
            request.uri.port = subURI.port;

            std::string outMsg = Parser::encode(request, CORE_TOPICS::SUB_REQUEST);

            if (publicServer.sendTo(outMsg, uri) < 0)
            {
                // std::cerr << "Failed to send message to Publisher" << std::endl;
                Console::log(LogLevel::ERROR, "Failed to send Request message to Publisher at " + uri.toString());
                outgoingRequests.push(uri);
            }
            else
            {
                awaitingResponses.insert(uri);
            }
        }

        // Read any incoming responses
        if (awaitingResponses.size() > 0)
        {
            std::string inMsg;
            URI sender;
            if (publicServer.receiveFrom(inMsg, MAX_MSG_SIZE, sender) < 0)
            {
                // std::cerr << "Failed to receive message from Publisher" << std::endl;
                Console::log(LogLevel::ERROR, "Failed to receive Response message from Publisher at " + sender.toString());
            }
            else if (inMsg.size() > 0 && awaitingResponses.find(sender) != awaitingResponses.end())
            {
                awaitingResponses.erase(sender);

                // Parse Response message
                private_msgs::Response response;
                if (!Parser::decode(inMsg, response))
                {
                    // std::cerr << "Failed to decode message from Publisher" << std::endl;
                    Console::log(LogLevel::ERROR, "Failed to decode Response message from Publisher at " + sender.toString());
                }
                else if (response.topic.data == topic && response.error.data.size() == 0)
                {
                    if (response.protocol.data != "TCP")
                    {
                        // std::cerr << "Unsupported protocol" << std::endl;
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
                URI subURI = publicServer.getURI();
                uri.uri.hostname = subURI.ip;
                uri.uri.port = subURI.port;
                std::string outMsg = Parser::encode(uri, CORE_TOPICS::SUB_RESPONSE);
                pair.second->send(outMsg);
                pubs.insert(pair);
            }
            else {
                int ret = pair.second->connect();
                if (ret < 0)
                {
                    // std::cerr << "Failed to connect to Publisher" << std::endl;
                    Console::log(LogLevel::ERROR, "Failed to connect to Publisher at " + pair.first.toString());
                    awaitingConnect.push(pair);
                }
                else if (ret == 1) {
                    Console::log(LogLevel::WARN, "Connection to Publisher at " + pair.first.toString() + " is pending");
                    awaitingConnect.push(pair);
                }
                else if (ret == 0  || ret == 2)
                {
                    private_msgs::URIStamped uri;
                    URI subURI = publicServer.getURI();
                    uri.uri.hostname = subURI.ip;
                    uri.uri.port = subURI.port;
                    std::string outMsg = Parser::encode(uri, CORE_TOPICS::SUB_RESPONSE);
                    pair.second->send(outMsg);
                    pubs.insert(pair);
                }
            }
        }

        // Read from any connected Publishers
        for (auto &pub : pubs)
        {
            std::string inMsg;
            if (pub.second->receive(inMsg, MAX_MSG_SIZE) < 0)
            {
                // std::cerr << "Failed to receive message from Publisher" << std::endl;
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