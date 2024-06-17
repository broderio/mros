#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"
#include "messages/geometry_msgs/quaternion.hpp"
#include "messages/geometry_msgs/vector3.hpp"

namespace geometry_msgs {

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
    String child_frame_id;
    Transform transform;

    TransformStamped() : header(), transform() {}

    TransformStamped(const Header& header, const Transform& transform) : header(header), transform(transform) {}

    TransformStamped(const TransformStamped& other) : header(other.header), transform(other.transform) {}

    TransformStamped& operator=(const TransformStamped& other) {
        if (this == &other) {
            return *this;
        }
        header = other.header;
        transform = other.transform;
        return *this;
    }

    uint16_t getMsgLen() const override {
        return header.getMsgLen() + child_frame_id.getMsgLen() + transform.getMsgLen();
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append(header.encode());
        msg.append(child_frame_id.encode());
        msg.append(transform.encode());
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a PointStamped." << std::endl;
            return;
        }

        int len = 0;
        header.decode(msg); len += header.getMsgLen();
        child_frame_id.decode(msg.substr(len, msg.find_first_of('\0', len) - len)); len += child_frame_id.getMsgLen();
        transform.decode(msg.substr(header.getMsgLen()));
    }
};

class TF : IMessage {
public:
    std::vector<TransformStamped> transforms;

    TF() {}

    TF(const std::vector<TransformStamped>& transforms) : transforms(transforms) {}

    TF(const TF& other) : transforms(other.transforms) {}

    TF& operator=(const TF& other) {
        if (this == &other) {
            return *this;
        }
        transforms = other.transforms;
        return *this;
    }

    uint16_t getMsgLen() const override {
        uint16_t len = sizeof(uint32_t);
        for (const TransformStamped& t : transforms) {
            len += t.getMsgLen();
        }
        return len;
    }

    virtual std::string encode() const override {
        std::string msg;
        for (const TransformStamped& t : transforms) {
            msg.append(t.encode());
        }
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < sizeof(uint32_t)) {
            std::cerr << "Error: message is too short to be a TF." << std::endl;
            return;
        }

        int len = 0;
        while (len < msg.size()) {
            TransformStamped t;
            t.decode(msg.substr(len));
            transforms.push_back(t);
            len += t.getMsgLen();
        }
    }
};

} // namespace geometry_msgs

#endif // TRANSFORM_HPP