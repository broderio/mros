#ifndef REQUEST_HPP
#define REQUEST_HPP

#include <messages/std_msgs/string.hpp>
#include <messages/std_msgs/header.hpp>

namespace private_msgs {

class Request : public IMessage {
public:
    Header header;
    String topic;
    String protocol;

    Request();

    Request(const Header& header, const String& topic, const String& protocol);

    Request(const Request& other);

    Request& operator=(const Request& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

}

#endif // REQUEST_HPP