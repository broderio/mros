#ifndef POSE2D_HPP
#define POSE2D_HPP

#include "messages/message.hpp"

class Pose2D : public IMessage {
    friend Parser;
public:
    int64_t utime;
    float x;
    float y;
    float theta;

    uint16_t getMsgLen() const override {
        return sizeof(int64_t) + 3 * sizeof(float);
    }

private:
    virtual std::string encode() const override {
        std::string msg;
        msg.append((char*)&utime, sizeof(int64_t));
        msg.append((char*)&x, sizeof(float));
        msg.append((char*)&y, sizeof(float));
        msg.append((char*)&theta, sizeof(float));
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a Pose2D." << std::endl;
            return;
        }

        std::memcpy(&utime, msg.data(), sizeof(utime));
        std::memcpy(&x, msg.data() + sizeof(utime), sizeof(x));
        std::memcpy(&y, msg.data() + sizeof(utime) + sizeof(x), sizeof(y));
        std::memcpy(&theta, msg.data() + sizeof(utime) + sizeof(x) + sizeof(y), sizeof(theta));
    }
};

#endif // POSE2D_HPP