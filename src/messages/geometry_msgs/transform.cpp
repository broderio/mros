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
    std::stringstream ss;
    ss << "translation:\n" << addTab(translation.toString(), 1) << '\n';
    ss << "rotation:\n" << addTab(rotation.toString(), 1);
    return ss.str();
}

std::string Transform::encode() const {
    std::string msg;
    msg.append(translation.encode());
    msg.append(rotation.encode());
    return msg;
}

bool Transform::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Transform." << std::endl;
        return false;
    }

    int len = 0;
    if (!translation.decode(msg)) {
        std::cerr << "Error: failed to decode translation." << std::endl;
        return false;
    }
    len += translation.getMsgLen();

    if (!rotation.decode(msg.substr(len))) {
        std::cerr << "Error: failed to decode rotation." << std::endl;
        return false;
    }

    return true;
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
    std::stringstream ss;
    ss << "header:\n" << addTab(header.toString(), 1) << '\n';
    ss << "child_frame_id: " << addTab(child_frame_id.toString(), 1) << '\n';
    ss << "transform:\n" << addTab(transform.toString(), 1);
    return ss.str();
}

std::string TransformStamped::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(child_frame_id.encode());
    msg.append(transform.encode());
    return msg;
}

bool TransformStamped::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a TransformStamped." << std::endl;
        return false;
    }

    int len = 0;
    if (!header.decode(msg)) {
        std::cerr << "Error: failed to decode header." << std::endl;
        return false;
    }
    len += header.getMsgLen();

    if (!child_frame_id.decode(msg.substr(len))) {
        std::cerr << "Error: failed to decode child_frame_id." << std::endl;
        return false;
    }
    len += child_frame_id.getMsgLen();

    if (!transform.decode(msg.substr(len))) {
        std::cerr << "Error: failed to decode transform." << std::endl;
        return false;
    }

    return true;
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
    return IMessage::getVectorLen(transforms);
}

std::string TF::toString() const {
    std::stringstream ss;
    ss << "transforms: [\n";
    for (const auto& transform : transforms) {
        ss << addTab(transform.toString(), 1) << ",\n";
    }
    ss << "]";
    return ss.str();
}

std::string TF::encode() const {
    std::string msg;
    return IMessage::encodeVector(transforms);
}

bool TF::decode(const std::string& msg) {
    if (msg.size() < sizeof(uint32_t)) {
        std::cerr << "Error: message is too short to be a TF." << std::endl;
        return false;
    }

    if (!IMessage::decodeVector(transforms, msg)) {
        std::cerr << "Error: failed to decode transforms vector." << std::endl;
        return false;
    }

    return true;
}

} // namespace geometry_msgs