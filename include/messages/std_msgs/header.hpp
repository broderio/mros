#pragma once

#include "messages/message.hpp"
#include "messages/std_msgs/time.hpp"
#include "messages/std_msgs/string.hpp"
#include "messages/std_msgs/integer.hpp"

namespace std_msgs
{

    class Header : public IMessage
    {
    public:
        UInt32 seq;
        Time stamp;
        String frame_id;

        Header();

        Header(UInt32 seq, const Time &stamp, const String &frame_id);

        Header(const Header &other);

        Header &operator=(const Header &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

} // namespace std_msgs