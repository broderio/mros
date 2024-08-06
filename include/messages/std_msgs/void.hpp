#pragma once

#include "messages/message.hpp"

namespace std_msgs
{

    class Void : public IMessage
    {
    public:
        Void();

        Void(const Void &other);

        operator std::string() const;

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

}