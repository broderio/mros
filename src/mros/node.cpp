#include "mros/node.hpp"

namespace mros
{
    Node::Node(const std::string &name, const URI &coreURI)
        : name(name), coreURI(coreURI), client(), signalHandler()
    {
    }

    Node::~Node()
    {
        shutdown();
    }

    void Node::spin()
    {
        std::thread t(&Node::run, this);
        t.join();
    }

    void Node::spinOnce()
    {
        runOnce();
    }

    void Node::shutdown()
    {
        for (auto &topic : publishers)
        {
            for (auto &pub : topic.second)
            {
                contactMediator(topic.first, pub->msgType, pub->publicServer.getURI(), CORE_TOPICS::PUB_DEREGISTER);
            }
        }
        publishers.clear();

        for (auto &topic : subscribers)
        {
            for (auto &sub : topic.second)
            {
                contactMediator(topic.first, sub->msgType, sub->publicServer.getURI(), CORE_TOPICS::SUB_DEREGISTER);
            }
        }
        subscribers.clear();

        client.close();
    }

    bool Node::ok()
    {
        return signalHandler.ok();
    }

    void Node::run()
    {
        while (ok())
        {
            runOnce();
        }
    }

    void Node::runOnce()
    {
         // Check if any messages have been sent to our UDP server
        std::string inMsg;
        URI sender;
        if (client.receiveFrom(inMsg, MAX_MSG_SIZE, sender) < 0)
        {
            std::cerr << "Failed to receive message from Mediator" << std::endl;
        }

        // If we have a message, parse it
        if (inMsg.size() > 0)
        {
            uint16_t topicId;
            // Parse message based on topic
            if (Parser::getTopicID(inMsg, topicId))
            {

                private_msgs::Notify notify;
                private_msgs::Disconnect disconnect;
                if (topicId == CORE_TOPICS::PUB_NOTIFY || topicId == CORE_TOPICS::SUB_NOTIFY) {
                    if (!Parser::decode(inMsg, notify))
                    {
                        std::cerr << "Error: Could not decode message" << std::endl;
                        return;
                    }
                }
                else if (topicId == CORE_TOPICS::PUB_DISCONNECT || topicId == CORE_TOPICS::SUB_DISCONNECT) {
                    if (!Parser::decode(inMsg, disconnect))
                    {
                        std::cerr << "Error: Could not decode message" << std::endl;
                        return;
                    }
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

        // Run each publisher and subscriber
        for (auto &topic : publishers)
        {
            for (auto &pub : topic.second)
            {
                pub->runOnce();
            }
        }

        for (auto &topic : subscribers)
        {
            for (auto &sub : topic.second)
            {
                sub->runOnce();
            }
        }
    }

    bool Node::contactMediator(const std::string &topic, const std::string &msgType, const URI &publicURI, const uint16_t &id)
    {
        private_msgs::Register msg;
        Header header;
        header.stamp.sec = getTimeNano() / 1000000000;
        header.stamp.nsec = getTimeNano() % 1000000000;
        header.frame_id = name;
        msg.header = header;
        msg.topic = topic;
        msg.msgType = msgType;
        msg.uri = private_msgs::URI(publicURI.ip, publicURI.port);

        std::cout << publicURI.toString() << std::endl;

        client.sendTo(Parser::encode(msg, id), coreURI);

        return true;
    }

    void Node::handlePubNotify(const private_msgs::Notify &notify)
    {
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