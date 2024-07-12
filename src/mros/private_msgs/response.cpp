#include <mros/private_msgs/response.hpp>

namespace private_msgs
{

    Response::Response() : header(), error(), protocol(), topic(), uri() {}

    Response::Response(const Header &header, const String &error, const String &protocol, const String &topic, const URI &uri)
        : header(header), error(error), protocol(protocol), topic(topic), uri(uri) {}

    Response::Response(const Response &other)
    {
        header = other.header;
        error = other.error;
        protocol = other.protocol;
        topic = other.topic;
        uri = other.uri;
    }

    Response &Response::operator=(const Response &other)
    {
        if (this == &other)
        {
            return *this;
        }
        header = other.header;
        error = other.error;
        protocol = other.protocol;
        topic = other.topic;
        uri = other.uri;
        return *this;
    }

    uint16_t Response::getMsgLen() const
    {
        return header.getMsgLen() + error.getMsgLen() + protocol.getMsgLen() + topic.getMsgLen() + uri.getMsgLen();
    }

    std::string Response::toString() const
    {
        std::stringstream ss;
        ss << "header:\n";
        ss << addTab(header.toString(), 1) << "\n";
        ss << "error:\n";
        ss << addTab(error.toString(), 1) << "\n";
        ss << "protocol:\n";
        ss << addTab(protocol.toString(), 1) << "\n";
        ss << "topic:\n";
        ss << addTab(topic.toString(), 1) << "\n";
        ss << "uri:\n";
        ss << addTab(uri.toString(), 1) << "\n";
        return ss.str();
    }

    std::string Response::encode() const
    {
        std::string msg;
        msg.append(header.encode());
        msg.append(error.encode());
        msg.append(protocol.encode());
        msg.append(topic.encode());
        msg.append(uri.encode());
        return msg;
    }

    bool Response::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a Response." << std::endl;
            return false;
        }

        int len = 0;
        if (!header.decode(msg))
        {
            std::cerr << "Error: could not decode header." << std::endl;
            return false;
        }
        len += header.getMsgLen();

        if (!error.decode(msg.substr(len)))
        {
            std::cerr << "Error: could not decode error." << std::endl;
            return false;
        }
        len += error.getMsgLen();

        if (!protocol.decode(msg.substr(len)))
        {
            std::cerr << "Error: could not decode protocol." << std::endl;
            return false;
        }
        len += protocol.getMsgLen();

        if (!topic.decode(msg.substr(len)))
        {
            std::cerr << "Error: could not decode topic." << std::endl;
            return false;
        }
        len += topic.getMsgLen();

        if (!uri.decode(msg.substr(len)))
        {
            std::cerr << "Error: could not decode uri." << std::endl;
            return false;
        }

        return true;
    }

}