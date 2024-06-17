#ifndef STRING_HPP
#define STRING_HPP

#include "messages/message.hpp"

class String : public IMessage {
public:
    std::string data;

    String() : data("") {}

    String(const std::string& data) : data(data) {}

    String operator=(const std::string& data) {
        this->data = data;
        return *this;
    }

    operator std::string() const {
        return data;
    }

    uint16_t getMsgLen() const override {
        return data.size() + 1;
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append(data + '\0');
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a Pose2D." << std::endl;
            return;
        }

        data = msg;
    }
};

#endif // STRING_HPP