#include "messages/geometry_msgs/point.hpp"

using namespace std_msgs;

namespace geometry_msgs
{

    Point::Point() : x(0), y(0), z(0) {}

    Point::Point(Float32 x, Float32 y, Float32 z) : x(x), y(y), z(z) {}

    Point::Point(const Vector3 &vector) : x(vector.x), y(vector.y), z(vector.z) {}

    Point::Point(const Point &other) : x(other.x), y(other.y), z(other.z) {}

    Point &Point::operator=(const Point &other)
    {
        if (this == &other)
        {
            return *this;
        }
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    uint16_t Point::getMsgLen() const
    {
        return x.getMsgLen() + y.getMsgLen() + z.getMsgLen();
    }

    std::string Point::toString() const
    {
        std::stringstream ss;
        ss << "x: " << x.toString() << '\n';
        ss << "y: " << y.toString() << '\n';
        ss << "z: " << z.toString();
        return ss.str();
    }

    std::string Point::encode() const
    {
        std::string msg;
        msg.append(x.encode());
        msg.append(y.encode());
        msg.append(z.encode());
        return msg;
    }

    bool Point::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a Point." << std::endl;
            return false;
        }

        int len = 0;
        x.decode(msg); len += x.getMsgLen();
        y.decode(msg.substr(len)); len += y.getMsgLen();
        z.decode(msg.substr(len));
        return true;
    }

    PointStamped::PointStamped() {}

    PointStamped::PointStamped(const Header &header, const Point &point) : header(header), point(point) {}

    PointStamped::PointStamped(const PointStamped &other) : header(other.header), point(other.point) {}

    PointStamped &PointStamped::operator=(const PointStamped &other)
    {
        if (this == &other)
        {
            return *this;
        }
        header = other.header;
        point = other.point;
        return *this;
    }

    uint16_t PointStamped::getMsgLen() const
    {
        return header.getMsgLen() + point.getMsgLen();
    }

    std::string PointStamped::toString() const
    {
        std::stringstream ss;
        ss << "header:\n" << addTab(header.toString(), 1) << '\n';
        ss << "point:\n" << addTab(point.toString(), 1);
        return ss.str();
    }

    std::string PointStamped::encode() const
    {
        std::string msg;
        msg.append(header.encode());
        msg.append(point.encode());
        return msg;
    }

    bool PointStamped::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a PointStamped." << std::endl;
            return false;
        }

        int len = 0;
        if (!header.decode(msg)) {
            std::cerr << "Error: failed to decode header." << std::endl;
            return false;
        }
        len += header.getMsgLen();

        if (!point.decode(msg.substr(len))) {
            std::cerr << "Error: failed to decode point." << std::endl;
            return false;
        }

        return true;
    }

} // namespace geometry_msgs