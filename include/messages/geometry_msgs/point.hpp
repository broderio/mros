#pragma once

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"
#include "messages/std_msgs/float.hpp"
#include "messages/geometry_msgs/vector3.hpp"

using namespace std_msgs;

namespace geometry_msgs {

class Point : public IMessage {
public:
    Float32 x;
    Float32 y;
    Float32 z;

    Point();

    Point(Float32 x, Float32 y, Float32 z);

    Point(const Vector3& vector);

    Point(const Point& other);

    Point &operator=(const Point& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;
    
    bool decode(const std::string& msg) override;
};

class PointStamped : public IMessage {
public:
    Header header;
    Point point;

    PointStamped();

    PointStamped(const Header& header, const Point& point);

    PointStamped(const PointStamped& other);

    PointStamped& operator=(const PointStamped& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    bool decode(const std::string& msg) override;
    
};

} // namespace geometry_msgs