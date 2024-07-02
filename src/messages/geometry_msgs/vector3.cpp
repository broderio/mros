#include "messages/geometry_msgs/vector3.hpp"

using namespace std_msgs;

namespace geometry_msgs {

Vector3::Vector3() : x(0), y(0), z(0) {}

Vector3::Vector3(Float32 x, Float32 y, Float32 z) : x(x), y(y), z(z) {}

Vector3::Vector3(const Vector3& other) : x(other.x), y(other.y), z(other.z) {}

Vector3& Vector3::operator=(const Vector3& other) {
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
}

uint16_t Vector3::getMsgLen() const {
    return x.getMsgLen() + y.getMsgLen() + z.getMsgLen();
}

std::string Vector3::toString() const {
    std::stringstream ss;
    ss << "x: " << x.toString() << '\n';
    ss << "y: " << y.toString() << '\n';
    ss << "z: " << z.toString();
    return ss.str();
}

std::string Vector3::encode() const {
    std::string msg;
    msg.append(x.encode());
    msg.append(y.encode());
    msg.append(z.encode());
    return msg;
}

bool Vector3::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Vector3." << std::endl;
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

    return true;
}


Vector3Stamped::Vector3Stamped() : header(), vector() {}

Vector3Stamped::Vector3Stamped(const Header& header, const Vector3& vector) : header(header), vector(vector) {}

Vector3Stamped::Vector3Stamped(const Vector3Stamped& other) : header(other.header), vector(other.vector) {}

Vector3Stamped& Vector3Stamped::operator=(const Vector3Stamped& other) {
    header = other.header;
    vector = other.vector;
    return *this;
}

uint16_t Vector3Stamped::getMsgLen() const {
    return header.getMsgLen() + vector.getMsgLen();
}

std::string Vector3Stamped::toString() const {
    std::stringstream ss;
    ss << "header:\n" << addTab(header.toString(), 1) << '\n';
    ss << "vector:\n" << addTab(vector.toString(), 1);
    return ss.str();
}

std::string Vector3Stamped::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(vector.encode());
    return msg;
}

bool Vector3Stamped::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Vector3Stamped." << std::endl;
        return false;
    }

    int len = 0;
    if (!header.decode(msg)) {
        std::cerr << "Error: failed to decode header." << std::endl;
        return false;
    }
    len += header.getMsgLen();

    if (!vector.decode(msg.substr(len))) {
        std::cerr << "Error: failed to decode vector." << std::endl;
        return false;
    }

    return true;
}

} // namespace geometry_msgs