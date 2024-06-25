#include "mros/mediator.hpp"

using namespace mros;

Mediator::Mediator() : topicMap(), server(MEDIATOR_PORT_NUM) {}

Mediator::~Mediator() {}

void Mediator::spin() {
    std::thread t(&Mediator::run, this);
    t.join();
}

void Mediator::run() {
    while (true) {
        std::string inMsg;
        URI clientURI;
        int err = server.receiveFrom(inMsg, 1024, clientURI);
        if (err < 0) {
            std::cerr << "Error: server receiveFrom failed." << std::endl;
            return;
        }

        Header header;
        std::string msg;
        uint16_t topicId;
        Parser::getTopicID(inMsg, topicId);

        if (topicId == CORE_TOPICS::REGISTER) {
            private_msgs::Register registerMsg;
            Parser::decode(inMsg, registerMsg);
            parseRegisterMessage(registerMsg);
        } 
        else if (topicId == CORE_TOPICS::DEREGISTER) {
            private_msgs::Register deregisterMsg;
            Parser::decode(inMsg, deregisterMsg);
            parseDeregisterMessage(deregisterMsg);
        } 
        else 
        {
            std::cerr << "Error: unknown topic ID " << topicId << std::endl;
        }
    }
}

void Mediator::parseRegisterMessage(const private_msgs::Register& msg) {
    std::string topic = msg.topic.data;
    std::string msgType = msg.msgType.data;
    std::string role = msg.role.data;
    URI uri = msg.uri.toURI(); // URI that we will send response to

    std::vector<private_msgs::URI> uriArray; // URIs that will be sent in response

    if (role == "publisher") 
    {
        if (topicMap.find(topic) == topicMap.end()) 
        {
            topicMap[topic] = Topic();
            topicMap[topic].msgType = msgType;
        }
        topicMap[topic].publishers.insert(uri);
        uriArray.push_back(private_msgs::URI(uri.ip, uri.port));
    } 
    else if (role == "subscriber") 
    {
        if (topicMap.find(topic) == topicMap.end()) 
        {
            topicMap[topic] = Topic();
            topicMap[topic].msgType = msgType;
        }
        topicMap[topic].subscribers.insert(uri);
        for (const URI& publisher : topicMap[topic].publishers) 
        {
            uriArray.push_back(private_msgs::URI(publisher.ip, publisher.port));
        }
    } 
    else 
    {
        std::cerr << "Error: unknown role " << role << std::endl;
        return;
    }

    Time time; 
    int t = getTimeNano();
    time.sec =  t / 1000000000;
    time.nsec = t % 1000000000;

    Header header;
    header.stamp = time;

    private_msgs::URIArray response;
    response.header = header;
    response.uriArray = uriArray;

    std::string outMsg = Parser::encode(response, CORE_TOPICS::RESPONSE);
    server.sendTo(outMsg, uri);
}

void Mediator::parseDeregisterMessage(const private_msgs::Register& msg) {
    std::string topic = msg.topic.data;
    std::string msgType = msg.msgType.data;
    std::string role = msg.role.data;

    private_msgs::URI uriMsg = msg.uri; // URI that we will notify nodes about
    URI uri = uriMsg.toURI();

    std::vector<URI> uriArray; // URIs that will be sent a response

    if (topicMap.find(topic) == topicMap.end()) 
    {
        std::cerr << "Error: topic " << topic << " not found." << std::endl;
        return;
    }

    if (role == "publisher") 
    {
        if (topicMap[topic].publishers.find(uri) == topicMap[topic].publishers.end()) 
        {
            std::cerr << "Error: publisher " << uri.toString() << " not found." << std::endl;
            return;
        }
        topicMap[topic].publishers.erase(uri);
        for (const URI& subscriber : topicMap[topic].subscribers) 
        {
            uriArray.push_back(subscriber);
        }
    } 
    else if (role == "subscriber") 
    {
        if (topicMap[topic].subscribers.find(uri) == topicMap[topic].subscribers.end()) 
        {
            std::cerr << "Error: subscriber " << uri.toString() << " not found." << std::endl;
            return;
        }
        topicMap[topic].subscribers.erase(uri);
        for (const URI& publisher : topicMap[topic].publishers) 
        {
            uriArray.push_back(publisher);
        }
    } 
    else 
    {
        std::cerr << "Error: unknown role " << role << std::endl;
    }

    Time time;
    int t = getTimeNano();
    time.sec = t / 1000000000;
    time.nsec = t % 1000000000;

    Header header;
    header.stamp = time;

    private_msgs::Disconnect response;
    response.header = header;
    response.uri = uriMsg;
    response.topic = msg.topic;

    std::string outMsg = Parser::encode(response, CORE_TOPICS::DISCONNECT);
    for (const URI& uri : uriArray) 
    {
        server.sendTo(outMsg, uri);
    }
}