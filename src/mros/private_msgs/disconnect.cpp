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
    ss << "Disconnect:\n";
    ss << '\t' << header.toString() << "\n";
    ss << '\t' << uri.toString() << "\n";
    ss << '\t' << topic.toString() << "\n";
    return ss.str();
}

std::string Disconnect::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(uri.encode());
    msg.append(topic.encode());
    return msg;
}

void Disconnect::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Disconnect." << std::endl;
        return;
    }

    int len = 0;
    header.decode(msg); len += header.getMsgLen();
    uri.decode(msg.substr(len)); len += uri.getMsgLen();
    topic.decode(msg.substr(len));
}

}