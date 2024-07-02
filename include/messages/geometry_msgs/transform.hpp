#pragma once

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"
#include "messages/geometry_msgs/quaternion.hpp"
#include "messages/geometry_msgs/vector3.hpp"

using namespace std_msgs;

namespace geometry_msgs {

class Transform : public IMessage {
public:
    Vector3 translation;
    Quaternion rotation;

    Transform();

    Transform(const Vector3& translation, const Quaternion& rotation);

    Transform(const Transform& other);

    ~Transform();

    Transform& operator=(const Transform& other);
    
    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    bool decode(const std::string& msg) override;
};

class TransformStamped : public IMessage {
public:
    Header header;
    String child_frame_id;
    Transform transform;

    TransformStamped();

    TransformStamped(const Header& header, const Transform& transform);

    TransformStamped(const TransformStamped& other);

    ~TransformStamped();

    TransformStamped& operator=(const TransformStamped& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    bool decode(const std::string& msg) override;
};

class TF : IMessage {
public:
    std::vector<TransformStamped> transforms;

    TF();

    TF(const std::vector<TransformStamped>& transforms);

    TF(const TF& other);

    TF& operator=(const TF& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    bool decode(const std::string& msg) override;
};

} // namespace geometry_msgs