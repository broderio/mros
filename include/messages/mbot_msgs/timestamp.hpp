#pragma once

#include "messages/message.hpp"
#include "messages/std_msgs/integer.hpp"

namespace mbot_msgs
{

    class Timestamp : public IMessage
    {
    public:
        std_msgs::Int64 utime;

        Timestamp();

        Timestamp(std_msgs::Int64 utime);

        Timestamp(const Timestamp &other);

        Timestamp &operator=(const Timestamp &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

} // namespace std_msgs