#include "messages/mbot_msgs/pose2D.hpp"

namespace mbot_msgs
{
    Pose2D::Pose2D()
    {
    }

    Pose2D::Pose2D(std_msgs::Int64 utime, std_msgs::Float32 x, std_msgs::Float32 y, std_msgs::Float32 theta)
    : utime(utime), x(x), y(y), theta(theta) {}

    Pose2D::Pose2D(const Pose2D &other)
    : utime(other.utime), x(other.x), y(other.y), theta(other.theta) {}

    Pose2D &Pose2D::operator=(const Pose2D &other)
    {
        if (this == &other)
        {
            return *this;
        }

        utime = other.utime;
        x = other.x;
        y = other.y;
        theta = other.theta;

        return *this;
    }

    uint16_t Pose2D::getMsgLen() const
    {
        return utime.getMsgLen() + x.getMsgLen() + y.getMsgLen() + theta.getMsgLen();
    }

    std::string Pose2D::toString() const
    {
        std::stringstream ss;
        ss << "utime: " << utime.toString() << '\n';
        ss << "x: " << x.toString() << '\n';
        ss << "y: " << y.toString() << '\n';
        ss << "theta: " << theta.toString();
        return ss.str();
    }

    std::string Pose2D::encode() const
    {
        std::string msg;
        msg.append(utime.encode());
        msg.append(x.encode());
        msg.append(y.encode());
        msg.append(theta.encode());
        return msg;
    }

    bool Pose2D::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a Pose2D." << std::endl;
            return false;
        }

        int len = 0;
        if (!utime.decode(msg))
        {
            std::cerr << "Error: failed to decode utime." << std::endl;
            return false;
        }
        len += utime.getMsgLen();

        if (!x.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode x." << std::endl;
            return false;
        }
        len += x.getMsgLen();

        if (!y.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode y." << std::endl;
            return false;
        }
        len += y.getMsgLen();

        if (!theta.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode theta." << std::endl;
            return false;
        }

        return true;
    }

} // namespace std_msgs