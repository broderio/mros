#pragma once 

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"
#include "messages/std_msgs/float.hpp"

using namespace std_msgs;

namespace geometry_msgs {

class Quaternion : public IMessage {
public:
    Float32 x;
    Float32 y;
    Float32 z;
    Float32 w;

    Quaternion();

    Quaternion(Float32 x, Float32 y, Float32 z, Float32 w);

    Quaternion(const Quaternion& other);

    Quaternion& operator=(const Quaternion& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    bool decode(const std::string& msg) override;
};

class QuaternionStamped : public IMessage {
public:
    Header header;
    Quaternion q;

    QuaternionStamped();

    QuaternionStamped(const Header& header, const Quaternion& q);

    QuaternionStamped(const QuaternionStamped& other);

    QuaternionStamped& operator=(const QuaternionStamped& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    bool decode(const std::string& msg) override;
};

} // namespace geometry_msgs