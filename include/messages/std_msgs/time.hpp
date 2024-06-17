#ifndef TIME_HPP
#define TIME_HPP

#include "messages/message.hpp"

class Time : public IMessage {
public:
    int32_t sec;
    int32_t nsec;

    Time() : sec(0), nsec(0) {}

    Time(int32_t sec, int32_t nsec) : sec(sec), nsec(nsec) {}

    Time(const Time& other) : sec(other.sec), nsec(other.nsec) {}

    Time& operator=(const Time& other) {
        if (this == &other) {
            return *this;
        }
        sec = other.sec;
        nsec = other.nsec;
        return *this;
    }

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