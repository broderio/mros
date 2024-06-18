#include "messages/geometry_msgs/pose.hpp"

namespace geometry_msgs {

Pose::Pose() : position(), orientation() {}

Pose::Pose(const Point& position, const Quaternion& orientation) : position(position), orientation(orientation) {}

Pose::Pose(const Pose& other) : position(other.position), orientation(other.orientation) {}

Pose& Pose::operator=(const Pose& other) {
    if (this == &other) {
        return *this;
    }
    position = other.position;
    orientation = other.orientation;
    return *this;
}

uint16_t Pose::getMsgLen() const {
    return position.getMsgLen() + orientation.getMsgLen();
}

std::string Pose::toString() const {
    std::stringstream ss;
    ss << "Pose:\n";
    ss << position.toString();
    ss << orientation.toString();
    return ss.str();
}

std::string Pose::encode() const {
    std::string msg;
    msg.append(position.encode());
    msg.append(orientation.encode());
    return msg;
}

void Pose::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Point." << std::endl;
        return;
    }

    int len = 0;
    position.decode(msg);
    orientation.decode(msg.substr(position.getMsgLen()));
}


PoseStamped::PoseStamped() {}

PoseStamped::PoseStamped(const Header& header, const Pose& pose) : header(header), pose(pose) {}

PoseStamped::PoseStamped(const PoseStamped& other) : header(other.header), pose(other.pose) {}

PoseStamped& PoseStamped::operator=(const PoseStamped& other) {
    if (this == &other) {
        return *this;
    }
    header = other.header;
    pose = other.pose;
    return *this;
}

uint16_t PoseStamped::getMsgLen() const {
    return header.getMsgLen() + pose.getMsgLen();
}

std::string PoseStamped::toString() const {
    std::stringstream ss;
    ss << "PoseStamped:\n";
    ss << header.toString();
    ss << pose.toString();
    return ss.str();
}

std::string PoseStamped::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(pose.encode());
    return msg;
}

void PoseStamped::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a PointStamped." << std::endl;
        return;
    }

    header.decode(msg);
    pose.decode(msg.substr(header.getMsgLen()));
}

} // namespace geometry_msgs