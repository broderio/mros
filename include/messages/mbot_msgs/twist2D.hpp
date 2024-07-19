#pragma once

#include "messages/message.hpp"
#include "messages/std_msgs/integer.hpp"
#include "messages/std_msgs/float.hpp"

namespace mbot_msgs
{

    class Twist2D : public IMessage
    {
    public:
        std_msgs::Int64 utime;
        std_msgs::Float32 vx;
        std_msgs::Float32 vy;
        std_msgs::Float32 wz;

        Twist2D();

        Twist2D(std_msgs::Int64 utime, std_msgs::Float32 vx, std_msgs::Float32 vy, std_msgs::Float32 wz);

        Twist2D(const Twist2D &other);

        Twist2D &operator=(const Twist2D &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

} // namespace std_msgs