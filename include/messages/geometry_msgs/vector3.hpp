#ifndef VECTOR3_HPP
#define VECTOR3_HPP

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"

namespace geometry_msgs {

class Vector3 : public IMessage {
public:
    float x;
    float y;
    float z;

    Vector3();

    Vector3(float x, float y, float z);

    Vector3(const Vector3& other);

    Vector3& operator=(const Vector3& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

class Vector3Stamped : public IMessage {
public:
    Header header;
    Vector3 vector;

    Vector3Stamped();

    Vector3Stamped(const Header& header, const Vector3& vector);

    Vector3Stamped(const Vector3Stamped& other);

    Vector3Stamped& operator=(const Vector3Stamped& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

} // namespace geometry_msgs

#endif // VECTOR3_HPP