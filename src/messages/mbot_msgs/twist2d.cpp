#include "messages/mbot_msgs/twist2d.hpp"

Twist2d::Twist2d() : utime(0), vx(0), vy(0), wz(0) {}

Twist2d::Twist2d(int64_t utime, float vx, float vy, float wz) : utime(utime), vx(vx), vy(vy), wz(wz) {}

Twist2d::Twist2d(const Twist2d& other) : utime(other.utime), vx(other.vx), vy(other.vy), wz(other.wz) {}

Twist2d& Twist2d::operator=(const Twist2d& other) {
    if (this != &other) {
        utime = other.utime;
        vx = other.vx;
        vy = other.vy;
        wz = other.wz;
    }
    return *this;
}

uint16_t Twist2d::getMsgLen() const {
    return sizeof(int64_t) + 3 * sizeof(float);
}

std::string Twist2d::toString() const {
    std::stringstream ss;
    ss << "Twist2d: {";
    ss << "utime: " << utime << ", ";
    ss << "vx: " << vx << ", ";
    ss << "vy: " << vy << ", ";
    ss << "wz: " << wz << "}";
    return ss.str();
}

std::string Twist2d::encode() const {
    std::string msg;
    msg.append((char*)&utime, sizeof(int64_t));
    msg.append((char*)&vx, sizeof(float));
    msg.append((char*)&vy, sizeof(float));
    msg.append((char*)&wz, sizeof(float));
    return msg;
}

void Twist2d::decode(const std::string& msg) {
    if (msg.size() != getMsgLen()) {
        std::cerr << "Error: Invalid message size to Twist2D::decode()" << std::endl;
        return;
    }

    std::memcpy(&utime, msg.data(), sizeof(utime));
    std::memcpy(&vx, msg.data() + sizeof(utime), sizeof(vx));
    std::memcpy(&vy, msg.data() + sizeof(utime) + sizeof(vx), sizeof(vy));
    std::memcpy(&wz, msg.data() + sizeof(utime) + sizeof(vx) + sizeof(vy), sizeof(wz));
}