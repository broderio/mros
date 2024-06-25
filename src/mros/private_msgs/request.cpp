#include "mros/private_msgs/request.hpp"

namespace private_msgs {

Request::Request() : header(), topic(), protocol() {}

Request::Request(const Header& header, const String& topic, const String& protocol) 
: header(header), topic(topic), protocol(protocol) {}

Request::Request(const Request& other) {
    header = other.header;
    topic = other.topic;
    protocol = other.protocol;
}

Request& Request::operator=(const Request& other) {
    if (this == &other) {
        return *this;
    }

    header = other.header;
    topic = other.topic;
    protocol = other.protocol;
    return *this;
}

uint16_t Request::getMsgLen() const {
    return header.getMsgLen() + topic.getMsgLen() + protocol.getMsgLen();
}

std::string Request::toString() const {
    std::stringstream ss;
    ss << '\t' << header.toString() << "\n";
    ss << '\t' << topic.toString() << "\n";
    ss << '\t' << protocol.toString() << "\n";
    return ss.str();
}

std::string Request::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(topic.encode());
    msg.append(protocol.encode());
}

void Request::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Request." << std::endl;
        return;
    }

    int len = 0;
    header.decode(msg); len += header.getMsgLen();
    topic.decode(msg.substr(len)); len += topic.getMsgLen();
    protocol.decode(msg.substr(len));
}

}