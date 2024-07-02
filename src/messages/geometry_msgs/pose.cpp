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
    ss << "position:\n" << addTab(position.toString(), 1) << '\n';
    ss << "orientation:\n" << addTab(orientation.toString(), 1);
    return ss.str();
}

std::string Pose::encode() const {
    std::string msg;
    msg.append(position.encode());
    msg.append(orientation.encode());
    return msg;
}

bool Pose::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Pose." << std::endl;
        return false;
    }

    int len = 0;
    if (!position.decode(msg)) {
        std::cerr << "Error: failed to decode position." << std::endl;
        return false;
    }
    if (!orientation.decode(msg.substr(position.getMsgLen()))) {
        std::cerr << "Error: failed to decode orientation." << std::endl;
        return false;
    }

    return true;
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
    ss << "header:\n" << addTab(header.toString(), 1) << std::endl;
    ss << "pose:\n" << addTab(pose.toString(), 1);
    return ss.str();
}

std::string PoseStamped::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(pose.encode());
    return msg;
}

bool PoseStamped::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a PoseStamped." << std::endl;
        return false;
    }

    int len = 0;
    if (!header.decode(msg)) {
        std::cerr << "Error: failed to decode header." << std::endl;
        return false;
    }
    len += header.getMsgLen();

    if (!pose.decode(msg.substr(len))) {
        std::cerr << "Error: failed to decode pose." << std::endl;
        return false;
    }

    return true;
}

} // namespace geometry_msgs