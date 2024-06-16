#ifndef TIME_HPP
#define TIME_HPP

#include "messages/message.hpp"

class Time : public IMessage {
public:
    int32_t sec;
    int32_t nsec;

    uint16_t getMsgLen() const override {
        return 2 * sizeof(int32_t);
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append((char*)&sec, sizeof(int32_t));
        msg.append((char*)&nsec, sizeof(int32_t));
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a Pose2D." << std::endl;
            return;
        }

        int len = 0;
        std::memcpy(&sec, msg.data(), sizeof(sec)); len += sizeof(sec);
        std::memcpy(&nsec, msg.data() + len, sizeof(nsec));
    }
};

#endif // TIME_HPP