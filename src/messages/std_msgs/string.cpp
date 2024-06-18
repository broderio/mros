#include "messages/std_msgs/string.hpp"

String::String() : data("") {}

String::String(const std::string& data) : data(data) {}

String::String(const String& other) : data(other.data) {}

String String::operator=(const std::string& data) {
    this->data = data;
    return *this;
}

uint16_t String::getMsgLen() const {
    return data.size() + 1;
}

std::string String::toString() const {
    return data;
}

std::string String::encode() const {
    std::string msg;
    msg.append(data + '\0');
    return msg;
}

void String::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Pose2D." << std::endl;
        return;
    }

    data = msg;
}