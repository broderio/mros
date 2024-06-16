#ifndef TWIST2D_HPP
#define TWIST2D_HPP

#include "messages/message.hpp"

class Twist2D : public IMessage {
public:
    int64_t utime;
    float vx;
    float vy;
    float wz;

    uint16_t getMsgLen() const override {
        return sizeof(int64_t) + 3 * sizeof(float);
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append((char*)&utime, sizeof(int64_t));
        msg.append((char*)&vx, sizeof(float));
        msg.append((char*)&vy, sizeof(float));
        msg.append((char*)&wz, sizeof(float));
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() != getMsgLen()) {
            std::cerr << "Error: Invalid message size to Twist2D::decode()" << std::endl;
            return;
        }

        std::memcpy(&utime, msg.data(), sizeof(utime));
        std::memcpy(&vx, msg.data() + sizeof(utime), sizeof(vx));
        std::memcpy(&vy, msg.data() + sizeof(utime) + sizeof(vx), sizeof(vy));
        std::memcpy(&wz, msg.data() + sizeof(utime) + sizeof(vx) + sizeof(vy), sizeof(wz));
    }
};

#endif // TWIST2D_HPP