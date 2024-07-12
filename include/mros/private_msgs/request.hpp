#pragma once

#include <messages/std_msgs/string.hpp>
#include <messages/std_msgs/header.hpp>
#include <mros/private_msgs/uri.hpp>

using namespace std_msgs;

namespace private_msgs
{

    class Request : public IMessage
    {
    public:
        Header header;
        String topic;
        String protocol;
        URI uri;

        Request();

        Request(const Header &header, const String &topic, const String &protocol, const URI &uri);

        Request(const Request &other);

        Request &operator=(const Request &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

}