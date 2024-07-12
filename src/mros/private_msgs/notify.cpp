#include "mros/private_msgs/notify.hpp"

namespace private_msgs {

Notify::Notify() {}

Notify::Notify(const Header& header, const String& error, const String& topic, const std::vector<URI>& uris)
 : header(header), error(error), topic(topic), uris(uris) {}

Notify::Notify(const Notify& other)
 : header(other.header), error(other.error), topic(other.topic), uris(other.uris) {}

Notify& Notify::operator=(const Notify& other) {
    if (this == &other) {
        return *this;
    }
    header = other.header;
    error = other.error;
    topic = other.topic;
    uris = other.uris;
    return *this;
}

uint16_t Notify::getMsgLen() const {
    return header.getMsgLen() + error.getMsgLen() + topic.getMsgLen() + IMessage::getVectorLen(uris);
}

std::string Notify::toString() const {
    std::stringstream ss;
    ss << "header:\n" << addTab(header.toString(), 1) << "\n";
    ss << "error:\n" << addTab(error.toString(), 1) << "\n";
    ss << "topic:\n" << addTab(topic.toString(), 1) << "\n";
    ss << "uris:\n";
    for (const auto& uri : uris) {
        ss << addTab(uri.toString(), 1);
    }
    return ss.str();
}

std::string Notify::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(error.encode());
    msg.append(topic.encode());
    msg.append(IMessage::encodeVector(uris));
    return msg;
}

bool Notify::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Notify." << std::endl;
        return false;
    }

    int len = 0;
    if (!header.decode(msg)) {
        std::cerr << "Error: could not decode header." << std::endl;
        return false;
    }
    len += header.getMsgLen();

    if (!error.decode(msg.substr(len))) {
        std::cerr << "Error: could not decode error." << std::endl;
        return false;
    }
    len += error.getMsgLen();

    if (!topic.decode(msg.substr(len))) {
        std::cerr << "Error: could not decode topic." << std::endl;
        return false;
    }
    len += topic.getMsgLen();

    if (!IMessage::decodeVector(uris, msg.substr(len))) {
        std::cerr << "Error: could not decode uris." << std::endl;
        return false;
    }

    return true;
}

} // namespace private_msgs

