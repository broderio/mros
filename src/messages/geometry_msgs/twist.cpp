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
    ss << "linear:\n" << addTab(linear.toString(), 1) << '\n';
    ss << "angular:\n" << addTab(angular.toString(), 1);
    return ss.str();
}

std::string Twist::encode() const {
    std::string msg;
    msg.append(linear.encode());
    msg.append(angular.encode());
    return msg;
}

bool Twist::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Twist." << std::endl;
        return false;
    }

    int len = 0;
    if (!linear.decode(msg)) {
        std::cerr << "Error: failed to decode linear." << std::endl;
        return false;
    }
    len += linear.getMsgLen();

    if (!angular.decode(msg.substr(len))) {
        std::cerr << "Error: failed to decode angular." << std::endl;
        return false;
    }

    return true;
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
    ss << "header:\n" << addTab(header.toString(), 1) << '\n';
    ss << "twist:\n" << addTab(twist.toString(), 1);
    return ss.str();
}

std::string TwistStamped::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(twist.encode());
    return msg;
}

bool TwistStamped::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a TwistStamped." << std::endl;
        return false;
    }

    int len = 0;
    if (!header.decode(msg)) {
        std::cerr << "Error: failed to decode header." << std::endl;
        return false;
    }
    len += header.getMsgLen();

    if (!twist.decode(msg.substr(len))) {
        std::cerr << "Error: failed to decode twist." << std::endl;
        return false;
    }

    return true;
}

} // namespace geometry_msgs