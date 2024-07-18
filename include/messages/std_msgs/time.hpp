#pragma once

#include "messages/message.hpp"
#include "messages/std_msgs/integer.hpp"

namespace std_msgs
{

    class Time : public IMessage
    {
    public:
        Int32 sec;
        Int32 nsec;

        Time();

        Time(Int32 sec, Int32 nsec);

        Time(const Time &other);

        Time &operator=(const Time &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;

        static Time getTimeNow();
    };

}