#ifndef QUATERNION_MSG_HPP
#define QUATERNION_MSG_HPP

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"

namespace geometry_msgs {

class Quaternion : public IMessage {
public:
    float x;
    float y;
    float z;
    float w;

    Quaternion();

    Quaternion(float x, float y, float z, float w);

    Quaternion(const Quaternion& other);

    Quaternion& operator=(const Quaternion& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
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

    void decode(const std::string& msg) override;
};

} // namespace geometry_msgs

#endif // QUATERNION_MSG_HPP