#pragma once

#include "utils.hpp"

#include "messages/std_msgs/string.hpp"
#include "messages/std_msgs/header.hpp"
#include "mros/private_msgs/uri.hpp"

using namespace std_msgs;

namespace private_msgs {

class Notify : public IMessage {
public:
    Header header;
    String error;
    String topic;
    std::vector<URI> uris;

    Notify();

    Notify(const Header& header, const String& error, const String& topic, const std::vector<URI>& uris);

    Notify(const Notify& other);

    Notify& operator=(const Notify& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    bool decode(const std::string& msg) override;
};

} // namespace private_msgs