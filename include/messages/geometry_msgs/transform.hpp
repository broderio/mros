#ifndef POSE_HPP
#define POSE_HPP

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"
#include "messages/geometry_msgs/quaternion.hpp"
#include "messages/geometry_msgs/vector3.hpp"

class Transform : public IMessage {
public:
    Vector3 translation;
    Quaternion rotation;

    Transform() : translation(), rotation() {}

    Transform(const Vector3& translation, const Quaternion& rotation) : translation(translation), rotation(rotation) {}

    uint16_t getMsgLen() const override {
        return translation.getMsgLen() + rotation.getMsgLen();
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append(translation.encode());
        msg.append(rotation.encode());
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a Point." << std::endl;
            return;
        }

        int len = 0;
        translation.decode(msg);
        rotation.decode(msg.substr(translation.getMsgLen()));
    }
};

class TransformStamped : public IMessage {
public:
    Header header;
    Transform transform;

    uint16_t getMsgLen() const override {
        return header.getMsgLen() + transform.getMsgLen();
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append(header.encode());
        msg.append(transform.encode());
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a PointStamped." << std::endl;
            return;
        }

        header.decode(msg);
        transform.decode(msg.substr(header.getMsgLen()));
    }
};

#endif // POINT_HPP