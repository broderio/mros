#ifndef STRING_HPP
#define STRING_HPP

#include "messages/message.hpp"

class String : public IMessage {
public:
    std::string data;

    uint16_t getMsgLen() const override {
        return data.size();
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append(data);
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