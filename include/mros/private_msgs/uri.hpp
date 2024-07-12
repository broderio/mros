#pragma once

#include "utils.hpp"

#include "messages/std_msgs/string.hpp"
#include "messages/std_msgs/header.hpp"

using namespace std_msgs;

namespace private_msgs {

class URI : public IMessage {
public:
    String hostname;
    Int32 port;

    URI();

    URI(const std_msgs::String& hostname, const Int32& port);

    URI(const URI& other);

    URI& operator=(const URI& other);

    ::URI toURI() const;

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    bool decode(const std::string& msg) override;
};

class URIStamped : public IMessage {
public:
    Header header;
    URI uri;

    URIStamped();

    URIStamped(const Header& header, const URI& uri);

    URIStamped(const URIStamped& other);

    URIStamped& operator=(const URIStamped& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    bool decode(const std::string& msg) override;
};

}