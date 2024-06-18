#ifndef POSE_HPP
#define POSE_HPP

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"
#include "messages/geometry_msgs/quaternion.hpp"
#include "messages/geometry_msgs/point.hpp"

namespace geometry_msgs {

class Pose : public IMessage {
public:
    Point position;
    Quaternion orientation;

    Pose();

    Pose(const Point& position, const Quaternion& orientation);

    Pose(const Pose& other);

    Pose& operator=(const Pose& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

class PoseStamped : public IMessage {
public:
    Header header;
    Pose pose;

    PoseStamped();

    PoseStamped(const Header& header, const Pose& pose);

    PoseStamped(const PoseStamped& other);

    PoseStamped& operator=(const PoseStamped& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

} // namespace geometry_msgs

#endif // POINT_HPP