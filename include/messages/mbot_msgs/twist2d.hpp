#ifndef TWIST2D_HPP
#define TWIST2D_HPP

#include "messages/message.hpp"

class Twist2d : public IMessage {
public:
    int64_t utime;
    float vx;
    float vy;
    float wz;

    Twist2d();

    Twist2d(int64_t utime, float vx, float vy, float wz);

    Twist2d(const Twist2d& other);

    Twist2d& operator=(const Twist2d& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

#endif // TWIST2D_HPP