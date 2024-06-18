#include "messages/mbot_msgs/pose2d.hpp"

Pose2d::Pose2d() : utime(0), x(0), y(0), theta(0) {}

Pose2d::Pose2d(int64_t utime, float x, float y, float theta) : utime(utime), x(x), y(y), theta(theta) {}

Pose2d::Pose2d(const Pose2d& other) : utime(other.utime), x(other.x), y(other.y), theta(other.theta) {}

Pose2d& Pose2d::operator=(const Pose2d& other) {
    if (this == &other) {
        return *this;
    }
    utime = other.utime;
    x = other.x;
    y = other.y;
    theta = other.theta;
    return *this;
}

uint16_t Pose2d::getMsgLen() const {
    return sizeof(int64_t) + 3 * sizeof(float);
}

std::string Pose2d::toString() const {
    std::stringstream ss;
    ss << "Pose2D:\n";
    ss << '\t' << utime << "\n";
    ss << '\t' << x << "\n";
    ss << '\t' << y << "\n";
    ss << '\t' << theta << "\n";
    return ss.str();
}

std::string Pose2d::encode() const {
    std::string msg;
    msg.append((char*)&utime, sizeof(int64_t));
    msg.append((char*)&x, sizeof(float));
    msg.append((char*)&y, sizeof(float));
    msg.append((char*)&theta, sizeof(float));
    return msg;
}

void Pose2d::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Pose2D." << std::endl;
        return;
    }

    std::memcpy(&utime, msg.data(), sizeof(utime));
    std::memcpy(&x, msg.data() + sizeof(utime), sizeof(x));
    std::memcpy(&y, msg.data() + sizeof(utime) + sizeof(x), sizeof(y));
    std::memcpy(&theta, msg.data() + sizeof(utime) + sizeof(x) + sizeof(y), sizeof(theta));
}