#ifndef STRING_HPP
#define STRING_HPP

#include "messages/message.hpp"

class String : public IMessage {
public:
    std::string data;

    String();

    String(const std::string& data);

    String(const String& other);

    String operator=(const std::string& data);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

#endif // STRING_HPP