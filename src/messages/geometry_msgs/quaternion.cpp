#include "messages/geometry_msgs/quaternion.hpp"

using namespace std_msgs;

namespace geometry_msgs {

Quaternion::Quaternion() : x(0), y(0), z(0), w(0) {}

Quaternion::Quaternion(Float32 x, Float32 y, Float32 z, Float32 w) : x(x), y(y), z(z), w(w) {}

Quaternion::Quaternion(const Quaternion& other) : x(other.x), y(other.y), z(other.z), w(other.w) {}

Quaternion& Quaternion::operator=(const Quaternion& other) {
    if (this == &other) {
        return *this;
    }
    x = other.x;
    y = other.y;
    z = other.z;
    w = other.w;
    return *this;
}

uint16_t Quaternion::getMsgLen() const {
    return 4 * sizeof(float);
}

std::string Quaternion::toString() const {
    std::stringstream ss;
    ss << "x: " << x.toString() << '\n';
    ss << "y: " << y.toString() << '\n';
    ss << "z: " << z.toString() << '\n';
    ss << "w: " << w.toString() << '\n';
    return ss.str();
}

std::string Quaternion::encode() const {
    std::string msg;
    msg.append(x.encode());
    msg.append(y.encode());
    msg.append(z.encode());
    msg.append(w.encode());
    return msg;
}

bool Quaternion::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Quaternion." << std::endl;
        return false;
    }

    int len = 0;
    if (!x.decode(msg)) {
        std::cerr << "Error: failed to decode x." << std::endl;
        return false;
    }
    len += x.getMsgLen();

    if (!y.decode(msg.substr(len))) {
        std::cerr << "Error: failed to decode y." << std::endl;
        return false;
    }
    len += y.getMsgLen();

    if (!z.decode(msg.substr(len))) {
        std::cerr << "Error: failed to decode z." << std::endl;
        return false;
    }
    len += z.getMsgLen();

    if (!w.decode(msg.substr(len))) {
        std::cerr << "Error: failed to decode w." << std::endl;
        return false;
    }
    
    return true;
}

QuaternionStamped::QuaternionStamped() {}

QuaternionStamped::QuaternionStamped(const Header& header, const Quaternion& q) : header(header), q(q) {}

QuaternionStamped::QuaternionStamped(const QuaternionStamped& other) : header(other.header), q(other.q) {}

QuaternionStamped& QuaternionStamped::operator=(const QuaternionStamped& other) {
    if (this == &other) {
        return *this;
    }
    header = other.header;
    q = other.q;
    return *this;
}

uint16_t QuaternionStamped::getMsgLen() const {
    return header.getMsgLen() + q.getMsgLen();
}

std::string QuaternionStamped::toString() const {
    std::stringstream ss;
    ss << "header:\n" << addTab(header.toString(), 1) << std::endl;
    ss << "q:\n" << addTab(q.toString(), 1);
    return ss.str();
}

std::string QuaternionStamped::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(q.encode());
    return msg;
}

bool QuaternionStamped::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a QuaternionStamped." << std::endl;
        return false;
    }
    
    int len = 0;
    if (!header.decode(msg)) {
        std::cerr << "Error: failed to decode header." << std::endl;
        return false;
    }
    len += header.getMsgLen();

    if (!q.decode(msg.substr(len))) {
        std::cerr << "Error: failed to decode q." << std::endl;
        return false;
    }

    return true;
}

} // namespace geometry_msgs