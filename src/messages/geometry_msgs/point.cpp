#include "messages/geometry_msgs/point.hpp"

using namespace geometry_msgs;

Point::Point() : x(0), y(0), z(0) {}

Point::Point(float x, float y, float z) : x(x), y(y), z(z) {}

Point::Point(const Vector3& vector) : x(vector.x), y(vector.y), z(vector.z) {}

Point::Point(const Point& other) : x(other.x), y(other.y), z(other.z) {}

Point &Point::operator=(const Point& other) {
    if (this == &other) {
        return *this;
    }
    x = other.x;
    y = other.y;
    z = other.z;
    return *this;
}

uint16_t Point::getMsgLen() const {
    return 3 * sizeof(float);
}

std::string Point::toString() const {
    std::stringstream ss;
    ss << "Point:\n";
    ss << '\t' << x << "\n";
    ss << '\t' << y << "\n";
    ss << '\t' << z << "\n";
    return ss.str();
}

std::string Point::encode() const {
    std::string msg;
    msg.append((char*)&x, sizeof(float));
    msg.append((char*)&y, sizeof(float));
    msg.append((char*)&z, sizeof(float));
    return msg;
}

void Point::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Point." << std::endl;
        return;
    }

    int len = 0;
    std::memcpy(&x, msg.data(), sizeof(x)); len += sizeof(x);
    std::memcpy(&y, msg.data() + len, sizeof(y)); len += sizeof(y);
    std::memcpy(&z, msg.data() + len, sizeof(z));
}


PointStamped::PointStamped() {}

PointStamped::PointStamped(const Header& header, const Point& point) : header(header), point(point) {}

PointStamped::PointStamped(const PointStamped& other) : header(other.header), point(other.point) {}

PointStamped& PointStamped::operator=(const PointStamped& other) {
    if (this == &other) {
        return *this;
    }
    header = other.header;
    point = other.point;
    return *this;
}

uint16_t PointStamped::getMsgLen() const {
    return header.getMsgLen() + point.getMsgLen();
}

std::string PointStamped::toString() const {
    std::stringstream ss;
    ss << "PointStamped:\n";
    ss << header.toString();
    ss << point.toString();
    return ss.str();
}

std::string PointStamped::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(point.encode());
    return msg;
}

void PointStamped::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a PointStamped." << std::endl;
        return;
    }

    header.decode(msg);
    point.decode(msg.substr(header.getMsgLen()));
}