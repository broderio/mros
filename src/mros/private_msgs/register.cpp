#include "mros/private_msgs/register.hpp"

namespace private_msgs {

Register::Register() : header(), topic(), msgType(), role(), uri() {}

Register::Register(const Header& header, const String& topic, const String& msgType, const String& role, const URI& uri)
: header(header), topic(topic), msgType(msgType), role(role), uri(uri) {}

Register::Register(const Register& other) {
    header = other.header;
    topic = other.topic;
    msgType = other.msgType;
    role = other.role;
    uri = other.uri;
}

Register& Register::operator=(const Register& other) {
    if (this == &other) {
        return *this;
    }
    header = other.header;
    topic = other.topic;
    msgType = other.msgType;
    role = other.role;
    uri = other.uri;
    return *this;
}

uint16_t Register::getMsgLen() const {
    return header.getMsgLen() + topic.getMsgLen() + msgType.getMsgLen() + role.getMsgLen() + uri.getMsgLen();
}

std::string Register::toString() const {
    std::stringstream ss;
    ss << "Register:\n";
    ss << '\t' << header.toString() << "\n";
    ss << '\t' << topic.toString() << "\n";
    ss << '\t' << msgType.toString() << "\n";
    ss << '\t' << role.toString() << "\n";
    ss << '\t' << uri.toString() << "\n";
    return ss.str();
}

std::string Register::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(topic.encode());
    msg.append(msgType.encode());
    msg.append(role.encode());
    msg.append(uri.encode());
}

void Register::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Register." << std::endl;
        return;
    }

    int len = 0;
    header.decode(msg); len += header.getMsgLen();
    topic.decode(msg.substr(len)); len += topic.getMsgLen();
    msgType.decode(msg.substr(len)); len += msgType.getMsgLen();
    role.decode(msg.substr(len)); len += role.getMsgLen();
    uri.decode(msg.substr(len)); len += uri.getMsgLen();
}

}