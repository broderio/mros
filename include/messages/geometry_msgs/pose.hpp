#ifndef POSE_HPP
#define POSE_HPP

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"
#include "messages/geometry_msgs/quaternion.hpp"
#include "messages/geometry_msgs/point.hpp"

class Pose : public IMessage {
public:
    Point position;
    Quaternion orientation;

    Pose() : position(), orientation() {}

    Pose(const Point& position, const Quaternion& orientation) : position(position), orientation(orientation) {}

    uint16_t getMsgLen() const override {
        return position.getMsgLen() + orientation.getMsgLen();
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append(position.encode());
        msg.append(orientation.encode());
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a Point." << std::endl;
            return;
        }

        int len = 0;
        position.decode(msg);
        orientation.decode(msg.substr(position.getMsgLen()));
    }
};

class PoseStamped : public IMessage {
public:
    Header header;
    Pose pose;

    uint16_t getMsgLen() const override {
        return header.getMsgLen() + pose.getMsgLen();
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append(header.encode());
        msg.append(pose.encode());
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a PointStamped." << std::endl;
            return;
        }

        header.decode(msg);
        pose.decode(msg.substr(header.getMsgLen()));
    }
};

#endif // POINT_HPP