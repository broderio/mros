#include "mros/mediator.hpp"

using namespace mros;

Mediator::Mediator() : topicMap(), server(URI("0.0.0.0", MEDIATOR_PORT_NUM))
{
    server.bind();
    URI uri = server.getURI();
    std::cout << "Mediator URI: " << uri.toString() << std::endl;
}

Mediator::~Mediator()
{
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
            std::cerr << "Error: server receiveFrom failed." << std::endl;
            return;
        }
        else if (inMsg.size() == 0) {
            continue;
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
            std::cerr << "Error: unknown topic ID " << topicId << std::endl;
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
            error = "Error: topic " + topic + " already exists with different message type " + topicMap[topic].msgType;
            std::cerr << error << std::endl;
        }
        else if (topicMap[topic].pubs.find(uriPair) != topicMap[topic].pubs.end())
        {
            // Publisher already exists
            error = "Error: publisher " + uri.toString() + " already exists for topic " + topic;
            std::cerr << error << std::endl;
        }
    }

    // If no errors, add publisher to topic
    if (error.size() == 0)
    {
        // Add publisher to topic
        topicMap[topic].pubs.insert(uriPair);
        std::cout << "Registered publisher: " << uriPair.serverURI << " on topic " << topic << std::endl;

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
            error = "Error: topic " + topic + " already exists with different message type " + topicMap[topic].msgType;
            std::cerr << error << std::endl;
        }
        else if (topicMap[topic].subs.find(uriPair) != topicMap[topic].subs.end())
        {
            // Subscriber already exists
            error = "Error: subscriber " + uri.toString() + " already exists for topic " + topic;
            std::cerr << error << std::endl;
        }
    }

    // If no errors, add subscriber to topic
    if (error.size() == 0)
    {
        // Add subscriber to topic
        topicMap[topic].subs.insert(uriPair);
        std::cout << "Registered subscriber: " << uriPair.serverURI << " on topic " << topic << std::endl;

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
        std::cerr << "Error: topic " << topic << " not found." << std::endl;
        return;
    }

    // Check if publisher exists
    if (topicMap[topic].pubs.find(uriPair) == topicMap[topic].pubs.end())
    {
        std::cerr << "Error: publisher " << uri.toString() << " not found." << std::endl;
        return;
    }

    // Erase publisher from topic
    topicMap[topic].pubs.erase(uriPair);
    std::cout << "Deregistered publisher: " << uriPair.serverURI << " from topic " << topic << std::endl;

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
        std::cerr << "Error: topic " << topic << " not found." << std::endl;
        return;
    }

    // Check if subscriber exists
    if (topicMap[topic].subs.find(uriPair) == topicMap[topic].subs.end())
    {
        std::cerr << "Error: subscriber " << uri.toString() << " not found." << std::endl;
        return;
    }

    // Erase subscriber from topic
    topicMap[topic].subs.erase(uriPair);
    std::cout << "Deregistered subscriber: " << uriPair.serverURI << " from topic " << topic << std::endl;

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
    return clientURI < other.clientURI || serverURI < other.serverURI;
}