#include "mros/private_msgs/register.hpp"

namespace private_msgs {

Register::Register() : header(), topic(), msgType(), uri() {}

Register::Register(const Header& header, const String& topic, const String& msgType, const URI& uri)
: header(header), topic(topic), msgType(msgType), uri(uri) {}

Register::Register(const Register& other) {
    header = other.header;
    topic = other.topic;
    msgType = other.msgType;
    uri = other.uri;
}

Register& Register::operator=(const Register& other) {
    if (this == &other) {
        return *this;
    }
    header = other.header;
    topic = other.topic;
    msgType = other.msgType;
    uri = other.uri;
    return *this;
}

uint16_t Register::getMsgLen() const {
    return header.getMsgLen() + topic.getMsgLen() + msgType.getMsgLen()+ uri.getMsgLen();
}

std::string Register::toString() const {
    std::stringstream ss;
    ss << "header:\n" << addTab(header.toString(), 1) << "\n";
    ss << "topic:\n" << addTab(topic.toString(), 1) << "\n";
    ss << "msgType:\n" << addTab(msgType.toString(), 1) << "\n";
    ss << "uri:\n" << addTab(uri.toString(), 1) << "\n";
    return ss.str();
}

std::string Register::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(topic.encode());
    msg.append(msgType.encode());
    msg.append(uri.encode());
    return msg;
}

bool Register::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Register." << std::endl;
        return false;
    }

    int len = 0;
    if (!header.decode(msg)) {
        std::cerr << "Error: could not decode header." << std::endl;
        return false;
    }
    len += header.getMsgLen();

    if (!topic.decode(msg.substr(len))) {
        std::cerr << "Error: could not decode topic." << std::endl;
        return false;
    }
    len += topic.getMsgLen();

    if (!msgType.decode(msg.substr(len))) {
        std::cerr << "Error: could not decode msgType." << std::endl;
        return false;
    }
    len += msgType.getMsgLen();

    if (!uri.decode(msg.substr(len))) {
        std::cerr << "Error: could not decode uri." << std::endl;
        return false;
    }

    return true;
}

}