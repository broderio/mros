#ifndef REGISTER_MSG_HPP
#define REGISTER_MSG_HPP

#include <messages/std_msgs/string.hpp>
#include <messages/std_msgs/header.hpp>
#include "mros/private_msgs/uri.hpp"

namespace private_msgs {

class Register : public IMessage {
public:
    Header header;
    String topic;
    String msgType;
    String role;
    URI uri;

    Register();

    Register(const Header& header, const String& topic, const String& msgType, const String& role, const URI& uri);

    Register(const Register& other);

    Register& operator=(const Register& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

}

#endif // REGISTER_MSG_HPP