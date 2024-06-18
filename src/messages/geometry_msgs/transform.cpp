#include "messages/geometry_msgs/transform.hpp"

namespace geometry_msgs {

Transform::Transform() : translation(), rotation() {}

Transform::Transform(const Vector3& translation, const Quaternion& rotation) : translation(translation), rotation(rotation) {}

Transform::Transform(const Transform& other) : translation(other.translation), rotation(other.rotation) {}

Transform::~Transform() {}

Transform& Transform::operator=(const Transform& other) {
    if (this == &other) {
        return *this;
    }
    translation = other.translation;
    rotation = other.rotation;
    return *this;
}

uint16_t Transform::getMsgLen() const {
    return translation.getMsgLen() + rotation.getMsgLen();
}

std::string Transform::toString() const {
    std::string str;
    str.append("translation: ");
    str.append(translation.toString());
    str.append("\nrotation: ");
    str.append(rotation.toString());
    return str;
}

std::string Transform::encode() const {
    std::string msg;
    msg.append(translation.encode());
    msg.append(rotation.encode());
    return msg;
}

void Transform::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Point." << std::endl;
        return;
    }

    int len = 0;
    translation.decode(msg);
    rotation.decode(msg.substr(translation.getMsgLen()));
}


TransformStamped::TransformStamped() : header(), transform() {}

TransformStamped::TransformStamped(const Header& header, const Transform& transform) : header(header), transform(transform) {}

TransformStamped::TransformStamped(const TransformStamped& other) : header(other.header), transform(other.transform) {}

TransformStamped::~TransformStamped() {}

TransformStamped& TransformStamped::operator=(const TransformStamped& other) {
    if (this == &other) {
        return *this;
    }
    header = other.header;
    transform = other.transform;
    return *this;
}

uint16_t TransformStamped::getMsgLen() const {
    return header.getMsgLen() + child_frame_id.getMsgLen() + transform.getMsgLen();
}

std::string TransformStamped::toString() const {
    std::string str;
    str.append("header: ");
    str.append(header.toString());
    str.append("\nchild_frame_id: ");
    str.append(child_frame_id.toString());
    str.append("\ntransform: ");
    str.append(transform.toString());
    return str;
}

std::string TransformStamped::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(child_frame_id.encode());
    msg.append(transform.encode());
    return msg;
}

void TransformStamped::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a PointStamped." << std::endl;
        return;
    }

    int len = 0;
    header.decode(msg); len += header.getMsgLen();
    child_frame_id.decode(msg.substr(len, msg.find_first_of('\0', len) - len)); len += child_frame_id.getMsgLen();
    transform.decode(msg.substr(header.getMsgLen()));
}


TF::TF() {}

TF::TF(const std::vector<TransformStamped>& transforms) : transforms(transforms) {}

TF::TF(const TF& other) : transforms(other.transforms) {}

TF& TF::operator=(const TF& other) {
    if (this == &other) {
        return *this;
    }
    transforms = other.transforms;
    return *this;
}

uint16_t TF::getMsgLen() const {
    uint16_t len = sizeof(uint32_t);
    for (const TransformStamped& t : transforms) {
        len += t.getMsgLen();
    }
    return len;
}

std::string TF::toString() const {
    std::string str;
    for (const TransformStamped& t : transforms) {
        str.append(t.toString());
        str.append("\n");
    }
    return str;
}

std::string TF::encode() const {
    std::string msg;
    for (const TransformStamped& t : transforms) {
        msg.append(t.encode());
    }
    return msg;
}

void TF::decode(const std::string& msg) {
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

} // namespace geometry_msgs