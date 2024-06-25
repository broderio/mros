#ifndef DISCONNECT_MSG_HPP
#define DISCONNECT_MSG_HPP

#include "utils.hpp"

#include "messages/std_msgs/string.hpp"
#include "messages/std_msgs/header.hpp"

#include "mros/private_msgs/uri.hpp"

namespace private_msgs {

class Disconnect : public IMessage {
public:
    Header header;
    URI uri;
    String topic;

    Disconnect();

    Disconnect(const Header& header, const private_msgs::URI& uri, const String& topic);

    Disconnect(const Disconnect& other);

    Disconnect& operator=(const Disconnect& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

}

#endif // DISCONNECT_MSG_HPP