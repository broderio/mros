#include "mros/private_msgs/disconnect.hpp"

namespace private_msgs {

Disconnect::Disconnect() : header(), uri(), topic() {}

Disconnect::Disconnect(const Header& header, const private_msgs::URI& uri, const String& topic)
: header(header), uri(uri), topic(topic) {}

Disconnect::Disconnect(const Disconnect& other) {
    header = other.header;
    uri = other.uri;
    topic = other.topic;
}

Disconnect& Disconnect::operator=(const Disconnect& other) {
    if (this == &other) {
        return *this;
    }

    header = other.header;
    uri = other.uri;
    topic = other.topic;
    return *this;
}

uint16_t Disconnect::getMsgLen() const {
    return header.getMsgLen() + uri.getMsgLen() + topic.getMsgLen();
}

std::string Disconnect::toString() const {
    std::stringstream ss;
    ss << "header:\n" << addTab(header.toString(), 1) << "\n";
    ss << "uri:\n" << addTab(uri.toString(), 1) << "\n";
    ss << "topic:\n" << addTab(topic.toString(), 1) << "\n";
    return ss.str();
}

std::string Disconnect::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(uri.encode());
    msg.append(topic.encode());
    return msg;
}

bool Disconnect::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Disconnect." << std::endl;
        return false;
    }

    int len = 0;
    if (!header.decode(msg)) {
        std::cerr << "Error: could not decode header." << std::endl;
        return false;
    }
    len += header.getMsgLen();

    if (!uri.decode(msg.substr(len))) {
        std::cerr << "Error: could not decode uri." << std::endl;
        return false;
    }
    len += uri.getMsgLen();

    if (!topic.decode(msg.substr(len))) {
        std::cerr << "Error: could not decode topic." << std::endl;
        return false;
    }
    len += topic.getMsgLen();

    return true;
}

}