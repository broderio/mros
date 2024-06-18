#ifndef POSE2D_HPP
#define POSE2D_HPP

#include "messages/message.hpp"

class Pose2d : public IMessage {
public:
    int64_t utime;
    float x;
    float y;
    float theta;

    Pose2d();

    Pose2d(int64_t utime, float x, float y, float theta);

    Pose2d(const Pose2d& other);

    Pose2d& operator=(const Pose2d& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

#endif // POSE2D_HPP