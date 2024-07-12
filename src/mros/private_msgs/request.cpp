#include "mros/private_msgs/request.hpp"

namespace private_msgs
{

    Request::Request() : header(), topic(), protocol(), uri() {}

    Request::Request(const Header &header, const String &topic, const String &protocol, const URI &uri)
        : header(header), topic(topic), protocol(protocol), uri(uri) {}

    Request::Request(const Request &other)
    {
        header = other.header;
        topic = other.topic;
        protocol = other.protocol;
        uri = other.uri;
    }

    Request &Request::operator=(const Request &other)
    {
        if (this == &other)
        {
            return *this;
        }

        header = other.header;
        topic = other.topic;
        protocol = other.protocol;
        uri = other.uri;
        return *this;
    }

    uint16_t Request::getMsgLen() const
    {
        return header.getMsgLen() + topic.getMsgLen() + protocol.getMsgLen() + uri.getMsgLen();
    }

    std::string Request::toString() const
    {
        std::stringstream ss;
        ss << "header:\n"
           << addTab(header.toString(), 1) << "\n";
        ss << "topic:\n"
           << addTab(topic.toString(), 1) << "\n";
        ss << "protocol:\n"
           << addTab(protocol.toString(), 1) << "\n";
        ss << "uri:\n"
           << addTab(uri.toString(), 1) << "\n";
        return ss.str();
    }

    std::string Request::encode() const
    {
        std::string msg;
        msg.append(header.encode());
        msg.append(topic.encode());
        msg.append(protocol.encode());
        msg.append(uri.encode());
        return msg;
    }

    bool Request::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a Request." << std::endl;
            return false;
        }

        int len = 0;
        if (!header.decode(msg))
        {
            std::cerr << "Error: could not decode header." << std::endl;
            return false;
        }
        len += header.getMsgLen();

        if (!topic.decode(msg.substr(len)))
        {
            std::cerr << "Error: could not decode topic." << std::endl;
            return false;
        }
        len += topic.getMsgLen();

        if (!protocol.decode(msg.substr(len)))
        {
            std::cerr << "Error: could not decode protocol." << std::endl;
            return false;
        }
        len += protocol.getMsgLen();

        if (!uri.decode(msg.substr(len)))
        {
            std::cerr << "Error: could not decode uri." << std::endl;
            return false;
        }

        return true;
    }

}