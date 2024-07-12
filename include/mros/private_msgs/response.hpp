#pragma once 

#include "messages/std_msgs/string.hpp"
#include "messages/std_msgs/header.hpp"
#include "mros/private_msgs/uri.hpp"

using namespace std_msgs;

namespace private_msgs {

class Response : public IMessage {
public:
    Header header;
    String error;
    String protocol;
    String topic;
    URI uri;

    Response();

    Response(const Header& header, const String& error, const String& protocol, const String& topic, const URI& uri);

    Response(const Response& other);

    Response& operator=(const Response& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    bool decode(const std::string& msg) override;
};

}