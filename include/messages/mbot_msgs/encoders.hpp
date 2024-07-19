#pragma once

#include "messages/message.hpp"
#include "messages/std_msgs/integer.hpp"

namespace mbot_msgs
{

    class Encoders : public IMessage
    {
    public:
        std_msgs::Int64 utime;
        std_msgs::Int64 ticks[3];
        std_msgs::Int32 delta_ticks[3];
        std_msgs::Int32 delta_time;

        Encoders();

        Encoders(std_msgs::Int64 utime, std_msgs::Int64 ticks[3], std_msgs::Int32 delta_ticks[3], std_msgs::Int32 delta_time);

        Encoders(const Encoders &other);

        Encoders &operator=(const Encoders &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

} // namespace std_msgs