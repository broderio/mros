#pragma once

#include "messages/message.hpp"

namespace std_msgs
{

    class String : public IMessage
    {
    public:
        std::string data;

        String();

        String(const std::string &data);

        String(const String &other);

        String operator=(const std::string &data);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

}