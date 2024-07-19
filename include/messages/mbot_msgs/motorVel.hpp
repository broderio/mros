#pragma once

#include "messages/message.hpp"
#include "messages/std_msgs/integer.hpp"
#include "messages/std_msgs/float.hpp"

namespace mbot_msgs
{

    class MotorVel : public IMessage
    {
    public:
        std_msgs::Int64 utime;
        std_msgs::Float32 velocity[3];

        MotorVel();

        MotorVel(std_msgs::Int64 utime, std_msgs::Float32 velocity[3]);

        MotorVel(const MotorVel &other);

        MotorVel &operator=(const MotorVel &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

} // namespace std_msgs