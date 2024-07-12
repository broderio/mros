#pragma once 

#include <messages/std_msgs/string.hpp>
#include <messages/std_msgs/header.hpp>
#include "mros/private_msgs/uri.hpp"

using namespace std_msgs;

namespace private_msgs {

class Register : public IMessage {
public:
    Header header;
    String topic;
    String msgType;
    URI uri;

    Register();

    Register(const Header& header, const String& topic, const String& msgType, const URI& uri);

    Register(const Register& other);

    Register& operator=(const Register& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    bool decode(const std::string& msg) override;
};

}