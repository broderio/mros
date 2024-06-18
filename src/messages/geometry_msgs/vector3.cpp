#include "messages/geometry_msgs/vector3.hpp"

namespace geometry_msgs {

Vector3::Vector3() : x(0), y(0), z(0) {}

Vector3::Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

Vector3::Vector3(const Vector3& other) : x(other.x), y(other.y), z(other.z) {}

Vector3& Vector3::operator=(const Vector3& other) {
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
}

uint16_t Vector3::getMsgLen() const {
    return 3 * sizeof(float);
}

std::string Vector3::toString() const {
    std::stringstream ss;
    ss << "Vector3:\n";
    ss << '\t' << x << "\n";
    ss << '\t' << y << "\n";
    ss << '\t' << z << "\n";
    return ss.str();
}

std::string Vector3::encode() const {
    std::string msg;
    msg.append((char*)&x, sizeof(float));
    msg.append((char*)&y, sizeof(float));
    msg.append((char*)&z, sizeof(float));
    return msg;
}

void Vector3::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Point." << std::endl;
        return;
    }

    int len = 0;
    std::memcpy(&x, msg.data(), sizeof(x)); len += sizeof(x);
    std::memcpy(&y, msg.data() + len, sizeof(y)); len += sizeof(y);
    std::memcpy(&z, msg.data() + len, sizeof(z));
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
    ss << "Vector3Stamped:\n";
    ss << header.toString();
    ss << vector.toString();
    return ss.str();
}

std::string Vector3Stamped::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(vector.encode());
    return msg;
}

void Vector3Stamped::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a PointStamped." << std::endl;
        return;
    }

    header.decode(msg);
    vector.decode(msg.substr(header.getMsgLen()));
}

} // namespace geometry_msgs