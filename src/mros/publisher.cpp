#include "mros/publisher.hpp"

namespace mros
{
    Publisher::~Publisher()
    {
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
       shutdownFlag = true;
    }

    void Publisher::runOnce()
    {
        // Check for a new Subscriber Request
        if (awaitingRequests.size() > 0)
        {
            std::string inMsg;
            URI sender;
            if (publicServer.receiveFrom(inMsg, MAX_MSG_SIZE, sender) < 0)
            {
                std::cerr << "Error receiving message from Subscriber" << std::endl;
            }
            else if (inMsg.size() > 0 && awaitingRequests.find(sender) != awaitingRequests.end())
            {
                awaitingRequests.erase(sender);

                // Parse the Request message
                private_msgs::Request request;
                if (!Parser::decode(inMsg, request))
                {
                    std::cerr << "Failed to decode message from Subscriber" << std::endl;
                }
                else if (request.topic.data == topic)
                {
                    if (request.protocol.data != "TCP")
                    {
                        std::cerr << "Unsupported protocol" << std::endl;
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
            URI pubURI = privateServer.getURI();
            response.uri.hostname = pubURI.ip;
            response.uri.port = pubURI.port;

            std::string outMsg = Parser::encode(response, CORE_TOPICS::PUB_RESPONSE);

            if (publicServer.sendTo(outMsg, uri) < 0)
            {
                std::cerr << "Failed to send response to Subscriber" << std::endl;
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
            std::shared_ptr<TCPConnection> connection = std::make_shared<TCPConnection>();
            if (privateServer.accept(*connection) != 0)
            {
                std::cerr << "Error accepting connection from Subscriber" << std::endl;
            }
            // This is the TCP Client's URI not the Subscriber's public URI
            else{
                awaitingURI.push(connection);
            }
        }

        if (awaitingURI.size() > 0)
        {
            std::shared_ptr<TCPConnection> connection = awaitingURI.front();
            awaitingURI.pop();

            std::string inMsg;
            if (connection->receive(inMsg, MAX_MSG_SIZE) < 0)
            {
                std::cerr << "Error receiving message from Subscriber" << std::endl;
                awaitingURI.push(connection);
            }
            else if (inMsg.size() > 0)
            {
                private_msgs::URIStamped uriMsg;
                if (!Parser::decode(inMsg, uriMsg))
                {
                    std::cerr << "Failed to decode message from Subscriber" << std::endl;
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
            else {
                // Would block, push back onto queue
                awaitingURI.push(connection);
            }
        }
        
        // Send any outgoing messages
        if (msgQueue.size() > 0)
        {
            std::string msg = msgQueue.front();
            msgQueue.pop();
            for (auto &sub : subs)
            {
                sub.second->send(msg);
            }
        }
    }

    Publisher::Publisher(const std::string &topic, const size_t &queueSize, const std::string& msgType)
        : shutdownFlag(false), topic(topic), queueSize(queueSize), msgType(msgType), msgQueue(), publicServer(URI(getIPAddr(), 0)), privateServer(URI(getIPAddr(), 0)) {
            publicServer.bind();
            privateServer.bind();
            privateServer.listen();
        }

    std::shared_ptr<Publisher> Publisher::create(const std::string &topic, const size_t &queueSize, const std::string& msgType) {
        std::shared_ptr<Publisher> pubPtr(new Publisher(topic, queueSize, msgType));
        return pubPtr;
    }
}