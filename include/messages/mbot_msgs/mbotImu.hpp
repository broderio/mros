#pragma once

#include "messages/message.hpp"
#include "messages/std_msgs/integer.hpp"
#include "messages/std_msgs/float.hpp"

namespace mbot_msgs
{

    class MbotImu : public IMessage
    {
    public:
        std_msgs::Int64 utime;
        std_msgs::Float32 gyro[3];
        std_msgs::Float32 accel[3];
        std_msgs::Float32 mag[3];
        std_msgs::Float32 angles_rpy[3];
        std_msgs::Float32 angles_quat[4];
        std_msgs::Float32 temp;

        MbotImu();

        MbotImu(std_msgs::Int64 utime, std_msgs::Float32 gyro[3], std_msgs::Float32 accel[3], std_msgs::Float32 mag[3], std_msgs::Float32 angles_rpy[3], std_msgs::Float32 angles_quat[4], std_msgs::Float32 temp);

        MbotImu(const MbotImu &other);

        MbotImu &operator=(const MbotImu &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

} // namespace std_msgs