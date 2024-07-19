#pragma once

#include "messages/message.hpp"
#include "messages/std_msgs/integer.hpp"
#include "messages/std_msgs/float.hpp"

namespace mbot_msgs
{

    class Pose2D : public IMessage
    {
    public:
        std_msgs::Int64 utime;
        std_msgs::Float32 x;
        std_msgs::Float32 y;
        std_msgs::Float32 theta;

        Pose2D();

        Pose2D(std_msgs::Int64 utime, std_msgs::Float32 x, std_msgs::Float32 y, std_msgs::Float32 theta);

        Pose2D(const Pose2D &other);

        Pose2D &operator=(const Pose2D &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

} // namespace std_msgs