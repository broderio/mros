#include "mros/mediator.hpp"

using namespace mros;

Mediator::Mediator() : topicMap(), server(URI("0.0.0.0", MEDIATOR_PORT_NUM))
{
    Console::init("mediator");
    server.bind();
    
    std::vector<std::string> ipAddrs = getLocalIPv4Addresses();
    std::string ipList = "[";
    for (const std::string &ip : ipAddrs)
    {
        ipList += ip;
        if (ip != ipAddrs.back())
        {
            ipList += ", ";
        }
    }
    ipList += "]";
    Console::log(LogLevel::INFO, "Mediator started on IP addresses: " + ipList);
}

Mediator::~Mediator()
{
    std_msgs::String msg;
    msg.data = "Mediator shutting down";
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
}

void Mediator::spin()
{
    std::thread t(&Mediator::run, this);
    t.join();
}

void Mediator::run()
{
    while (signalHandler.ok())
    {
        std::string inMsg;
        URI clientURI;
        // std::cout << "Waiting for message..." << std::endl;
        int err = server.receiveFrom(inMsg, 1024, clientURI);
        if (err < 0)
        {
            Console::log(LogLevel::ERROR, "UDPServer::receiveFrom failed.");
            return;
        }
        else if (inMsg.size() == 0) {
            continue;
        }
        else if (inMsg.size() == 1) {
            if ((uint8_t)inMsg[0] == PING_MSG) {
                server.sendTo(std::string(1, PONG_MSG), clientURI);
                continue;
            }
            else if ((uint8_t)inMsg[0] == PONG_MSG) {
                Console::log(LogLevel::INFO, "Received PONG from " + clientURI.toString());
                continue;
            }
        }

        Header header;
        std::string msg;
        uint16_t topicId;
        Parser::getTopicID(inMsg, topicId);

        private_msgs::Register registerMsg;
        Parser::decode(inMsg, registerMsg);
        switch (topicId)
        {
        case CORE_TOPICS::PUB_REGISTER:
            registerPublisher(registerMsg, clientURI);
            break;
        case CORE_TOPICS::SUB_REGISTER:
            registerSubscriber(registerMsg, clientURI);
            break;
        case CORE_TOPICS::PUB_DEREGISTER:
            deregisterPublisher(registerMsg, clientURI);
            break;
        case CORE_TOPICS::SUB_DEREGISTER:
            deregisterSubscriber(registerMsg, clientURI);
            break;
        default:
            Console::log(LogLevel::ERROR, "Unknown topic ID: " + std::to_string(topicId));
            return;
        }
    }
}

void Mediator::registerPublisher(const private_msgs::Register &msg, const URI &clientURI)
{
    std::string topic = msg.topic.data;
    std::string msgType = msg.msgType.data;
    URI uri = msg.uri.toURI(); // URI that will be sent to existing subscribers

    std::vector<private_msgs::URI> subURIs; // URIs that will be sent to newly registered publisher
    std::string error = "";

    URIPair uriPair = {clientURI, uri};

    // Check if topic already exists
    if (topicMap.find(topic) == topicMap.end())
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
            error = "Topic " + topic + " already exists with different message type " + topicMap[topic].msgType;
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
        Console::log(LogLevel::INFO, "Registered publisher " + uriPair.serverURI.toString() + " from node " + uriPair.clientURI.toString() + " on topic " + topic);

        // Add existing subscribers to response
        for (const URIPair &subscriber : topicMap[topic].subs)
        {
            subURIs.push_back(private_msgs::URI(subscriber.serverURI.ip, subscriber.serverURI.port));
        }
    }

    Time time;
    int t = getTimeNano();
    time.sec = t / 1000000000;
    time.nsec = t % 1000000000;

    Header header;
    header.stamp = time;

    private_msgs::Notify response;
    response.header = header;
    response.error = error;
    response.topic = String(topic);
    response.uris = subURIs;

    // Send response to publisher
    std::string outMsg = Parser::encode(response, CORE_TOPICS::PUB_NOTIFY);
    server.sendTo(outMsg, uriPair.clientURI);

    // Send response to all subscribers of topic if no errors
    if (error.size() == 0 && topicMap[topic].subs.size() > 0)
    {
        response.uris.clear();
        response.uris.push_back(private_msgs::URI(uri.ip, uri.port));
        outMsg = Parser::encode(response, CORE_TOPICS::SUB_NOTIFY);
        for (const URIPair &subscriber : topicMap[topic].subs)
        {
            server.sendTo(outMsg, subscriber.clientURI);
        }
    }
}

void Mediator::registerSubscriber(const private_msgs::Register &msg, const URI &clientURI)
{
    std::string topic = msg.topic.data;
    std::string msgType = msg.msgType.data;
    URI uri = msg.uri.toURI(); // URI that will be sent to existing publishers

    std::vector<private_msgs::URI> pubURIs; // URIs that will be sent to newly registered subscriber
    std::string error = "";

    URIPair uriPair = {clientURI, uri};

    // Check if topic already exists
    if (topicMap.find(topic) == topicMap.end())
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
            error = "Topic " + topic + " already exists with different message type " + topicMap[topic].msgType;
            Console::log(LogLevel::ERROR, error);
        }
        else if (topicMap[topic].subs.find(uriPair) != topicMap[topic].subs.end())
        {
            // Subscriber already exists
            error = "Subscriber " + uri.toString() + " already exists for topic " + topic;
            Console::log(LogLevel::ERROR, error);
        }
    }

    // If no errors, add subscriber to topic
    if (error.size() == 0)
    {
        // Add subscriber to topic
        topicMap[topic].subs.insert(uriPair);
        Console::log(LogLevel::INFO, "Registered subscriber " + uriPair.serverURI.toString() + " from node " + uriPair.clientURI.toString() + " on topic " + topic);

        // Add existing publishers to response
        for (const URIPair &publisher : topicMap[topic].pubs)
        {
            pubURIs.push_back(private_msgs::URI(publisher.serverURI.ip, publisher.serverURI.port));
        }
    }

    Time time;
    int t = getTimeNano();
    time.sec = t / 1000000000;
    time.nsec = t % 1000000000;

    Header header;
    header.stamp = time;

    private_msgs::Notify response;
    response.header = header;
    response.error = error;
    response.topic = String(topic);
    response.uris = pubURIs;

    // Send response to subscriber
    std::string outMsg = Parser::encode(response, CORE_TOPICS::SUB_NOTIFY);
    server.sendTo(outMsg, uriPair.clientURI);

    // Send response to all publishers of topic if no errors
    if (error.size() == 0 && topicMap[topic].pubs.size() > 0)
    {
        response.uris.clear();
        response.uris.push_back(private_msgs::URI(uri.ip, uri.port));
        outMsg = Parser::encode(response, CORE_TOPICS::PUB_NOTIFY);
        for (const URIPair &publisher : topicMap[topic].pubs)
        {
            server.sendTo(outMsg, publisher.clientURI);
        }
    }
}

void Mediator::deregisterPublisher(const private_msgs::Register &msg, const URI &clientURI)
{
    std::string topic = msg.topic.data;
    std::string msgType = msg.msgType.data;
    URI uri = msg.uri.toURI(); // URI that will be deleted from map and sent to subscribers

    URIPair uriPair = {clientURI, uri};

    // Check if topic exists
    if (topicMap.find(topic) == topicMap.end())
    {
        Console::log(LogLevel::ERROR, "Topic " + topic + " not found.");
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
    Console::log(LogLevel::INFO, "Deregistered publisher " + uriPair.serverURI.toString() + " from node " + uriPair.clientURI.toString() + " on topic " + topic);

    Time time;
    int t = getTimeNano();
    time.sec = t / 1000000000;
    time.nsec = t % 1000000000;

    Header header;
    header.stamp = time;

    private_msgs::Disconnect response;
    response.header = header;
    response.uri = msg.uri;
    response.topic = msg.topic;

    // Send response to all subscribers of topic
    std::string outMsg = Parser::encode(response, CORE_TOPICS::SUB_DISCONNECT);
    for (const URIPair &subscriber : topicMap[topic].subs)
    {
        server.sendTo(outMsg, subscriber.clientURI);
    }
}

void Mediator::deregisterSubscriber(const private_msgs::Register &msg, const URI &clientURI)
{
    std::string topic = msg.topic.data;
    std::string msgType = msg.msgType.data;
    URI uri = msg.uri.toURI(); // URI that will be deleted from map and sent to publishers

    URIPair uriPair = {clientURI, uri};

    // Check if topic exists
    if (topicMap.find(topic) == topicMap.end())
    {
        Console::log(LogLevel::ERROR, "Topic " + topic + " not found.");
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
    Console::log(LogLevel::INFO, "Deregistered subscriber " + uriPair.serverURI.toString() + " from node " + uriPair.clientURI.toString() + " on topic " + topic);

    Time time;
    int t = getTimeNano();
    time.sec = t / 1000000000;
    time.nsec = t % 1000000000;

    Header header;
    header.stamp = time;

    private_msgs::Disconnect response;
    response.header = header;
    response.uri = msg.uri;
    response.topic = msg.topic;

    // Send response to all publishers of topic
    std::string outMsg = Parser::encode(response, CORE_TOPICS::PUB_DISCONNECT);
    for (const URIPair &publisher : topicMap[topic].pubs)
    {
        server.sendTo(outMsg, publisher.clientURI);
    }
}

bool Mediator::URIPair::operator<(const URIPair &other) const
{
    return clientURI < other.clientURI || (clientURI == other.clientURI && serverURI < other.serverURI);
}