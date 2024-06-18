#include "messages/geometry_msgs/twist.hpp"

namespace geometry_msgs {

Twist::Twist() : linear(), angular() {}

Twist::Twist(const Vector3 &linear, const Vector3 &angular) : linear(linear), angular(angular) {}

Twist::Twist(const Twist &other) : linear(other.linear), angular(other.angular) {}

Twist &Twist::operator=(const Twist &other) {
    if (this == &other) {
        return *this;
    }
    linear = other.linear;
    angular = other.angular;
    return *this;
}

uint16_t Twist::getMsgLen() const {
    return 3 * sizeof(float);
}

std::string Twist::toString() const {
    std::stringstream ss;
    ss << "Twist:\n";
    ss << linear.toString();
    ss << angular.toString();
    return ss.str();
}

std::string Twist::encode() const {
    std::string msg;
    msg.append(linear.encode());
    msg.append(angular.encode());
    return msg;
}

void Twist::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Point." << std::endl;
        return;
    }

    int len = 0;
    linear.decode(msg);
    angular.decode(msg.substr(linear.getMsgLen()));
}


TwistStamped::TwistStamped() : header(), twist() {}

TwistStamped::TwistStamped(const Header &header, const Twist &twist) : header(header), twist(twist) {}

TwistStamped::TwistStamped(const TwistStamped &other) : header(other.header), twist(other.twist) {}

TwistStamped &TwistStamped::operator=(const TwistStamped &other) {
    if (this == &other) {
        return *this;
    }
    header = other.header;
    twist = other.twist;
    return *this;
}

uint16_t TwistStamped::getMsgLen() const {
    return header.getMsgLen() + twist.getMsgLen();
}

std::string TwistStamped::toString() const {
    std::stringstream ss;
    ss << "TwistStamped:\n";
    ss << header.toString();
    ss << twist.toString();
    return ss.str();
}

std::string TwistStamped::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(twist.encode());
    return msg;
}

void TwistStamped::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a PointStamped." << std::endl;
        return;
    }

    header.decode(msg);
    twist.decode(msg.substr(header.getMsgLen()));
}

} // namespace geometry_msgs