#ifndef HEADER_HPP
#define HEADER_HPP

#include "messages/message.hpp"
#include "messages/std_msgs/time.hpp"
#include "messages/std_msgs/string.hpp"

class Header : public IMessage {
public:
    uint32_t seq;
    Time stamp;
    String frame_id;

    Header();

    Header(uint32_t seq, const Time& stamp, const String& frame_id);

    Header(const Header& other);

    Header& operator=(const Header& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

#endif // HEADER_HPP