#include "messages/geometry_msgs/quaternion.hpp"

namespace geometry_msgs {

Quaternion::Quaternion() : x(0), y(0), z(0), w(0) {}

Quaternion::Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

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
    ss << "Quaternion:\n";
    ss << '\t' << x << "\n";
    ss << '\t' << y << "\n";
    ss << '\t' << z << "\n";
    ss << '\t' << w << "\n";
    return ss.str();
}

std::string Quaternion::encode() const {
    std::string msg;
    msg.append((char*)&x, sizeof(float));
    msg.append((char*)&y, sizeof(float));
    msg.append((char*)&z, sizeof(float));
    msg.append((char*)&w, sizeof(float));
    return msg;
}

void Quaternion::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Point." << std::endl;
        return;
    }

    int len = 0;
    std::memcpy(&x, msg.data(), sizeof(x)); len += sizeof(x);
    std::memcpy(&y, msg.data() + len, sizeof(y)); len += sizeof(y);
    std::memcpy(&z, msg.data() + len, sizeof(z)); len += sizeof(z);
    std::memcpy(&w, msg.data() + len, sizeof(w));
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
    ss << "QuaternionStamped:\n";
    ss << header.toString();
    ss << q.toString();
    return ss.str();
}

std::string QuaternionStamped::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(q.encode());
    return msg;
}

void QuaternionStamped::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a PointStamped." << std::endl;
        return;
    }

    header.decode(msg);
    q.decode(msg.substr(q.getMsgLen()));
}

} // namespace geometry_msgs