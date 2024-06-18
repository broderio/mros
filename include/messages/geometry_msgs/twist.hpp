#ifndef TWIST_HPP
#define TWIST_HPP

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"
#include "messages/geometry_msgs/vector3.hpp"

namespace geometry_msgs {

class Twist : public IMessage {
public:
    Vector3 linear;
    Vector3 angular;

    Twist();

    Twist(const Vector3 &linear, const Vector3 &angular);

    Twist(const Twist &other);

    Twist &operator=(const Twist &other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

class TwistStamped : public IMessage {
public:
    Header header;
    Twist twist;

    TwistStamped();

    TwistStamped(const Header &header, const Twist &twist);

    TwistStamped(const TwistStamped &other);

    TwistStamped &operator=(const TwistStamped &other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

} // namespace geometry_msgs

#endif // TWIST_HPP