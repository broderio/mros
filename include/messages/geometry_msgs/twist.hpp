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

    Twist() : linear(), angular() {}

    Twist(const Vector3 &linear, const Vector3 &angular) : linear(linear), angular(angular) {}

    uint16_t getMsgLen() const override {
        return 3 * sizeof(float);
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append(linear.encode());
        msg.append(angular.encode());
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a Point." << std::endl;
            return;
        }

        int len = 0;
        linear.decode(msg);
        angular.decode(msg.substr(linear.getMsgLen()));
    }
};

class TwistStamped : public IMessage {
public:
    Header header;
    Twist twist;

    uint16_t getMsgLen() const override {
        return header.getMsgLen() + twist.getMsgLen();
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append(header.encode());
        msg.append(twist.encode());
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a PointStamped." << std::endl;
            return;
        }

        header.decode(msg);
        twist.decode(msg.substr(header.getMsgLen()));
    }
};

} // namespace geometry_msgs

#endif // TWIST_HPP