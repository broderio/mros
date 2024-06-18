#ifndef POINT_HPP
#define POINT_HPP

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"
#include "messages/geometry_msgs/vector3.hpp"

namespace geometry_msgs {

class Point : public IMessage {
public:
    float x;
    float y;
    float z;

    Point();

    Point(float x, float y, float z);

    Point(const Vector3& vector);

    Point(const Point& other);

    Point &operator=(const Point& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;
    
    void decode(const std::string& msg) override;
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

    void decode(const std::string& msg) override;
    
};

} // namespace geometry_msgs

#endif // POINT_HPP