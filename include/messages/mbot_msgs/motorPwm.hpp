#pragma once

#include "messages/message.hpp"
#include "messages/std_msgs/integer.hpp"
#include "messages/std_msgs/float.hpp"

namespace mbot_msgs
{

    class MotorPwm : public IMessage
    {
    public:
        std_msgs::Int64 utime;
        std_msgs::Float32 pwm[3];

        MotorPwm();

        MotorPwm(std_msgs::Int64 utime, std_msgs::Float32 pwm[3]);

        MotorPwm(const MotorPwm &other);

        MotorPwm &operator=(const MotorPwm &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

} // namespace std_msgs