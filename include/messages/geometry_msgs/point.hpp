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

    Point() : x(0), y(0), z(0) {}

    Point(float x, float y, float z) : x(x), y(y), z(z) {}

    Point(const Vector3& vector) : x(vector.x), y(vector.y), z(vector.z) {}

    uint16_t getMsgLen() const override {
        return 3 * sizeof(float);
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append((char*)&x, sizeof(float));
        msg.append((char*)&y, sizeof(float));
        msg.append((char*)&z, sizeof(float));
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a Point." << std::endl;
            return;
        }

        int len = 0;
        std::memcpy(&x, msg.data(), sizeof(x)); len += sizeof(x);
        std::memcpy(&y, msg.data() + len, sizeof(y)); len += sizeof(y);
        std::memcpy(&z, msg.data() + len, sizeof(z));
    }
};

class PointStamped : public IMessage {
public:
    Header header;
    Point point;

    uint16_t getMsgLen() const override {
        return header.getMsgLen() + point.getMsgLen();
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append(header.encode());
        msg.append(point.encode());
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a PointStamped." << std::endl;
            return;
        }

        header.decode(msg);
        point.decode(msg.substr(header.getMsgLen()));
    }
};

} // namespace geometry_msgs

#endif // POINT_HPP