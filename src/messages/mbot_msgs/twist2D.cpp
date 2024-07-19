#include "messages/mbot_msgs/twist2D.hpp"

namespace mbot_msgs
{
    Twist2D::Twist2D()
    {
    }

    Twist2D::Twist2D(std_msgs::Int64 utime, std_msgs::Float32 vx, std_msgs::Float32 vy, std_msgs::Float32 wz)
    : utime(utime), vx(vx), vy(vy), wz(wz) {}

    Twist2D::Twist2D(const Twist2D &other)
    : utime(other.utime), vx(other.vx), vy(other.vy), wz(other.wz) {}

    Twist2D &Twist2D::operator=(const Twist2D &other)
    {
        if (this == &other)
        {
            return *this;
        }

        this->utime = other.utime;
        this->vx = other.vx;
        this->vy = other.vy;
        this->wz = other.wz;

        return *this;
    }

    uint16_t Twist2D::getMsgLen() const
    {
        return utime.getMsgLen() + vx.getMsgLen() + vy.getMsgLen() + wz.getMsgLen();
    }

    std::string Twist2D::toString() const
    {
        std::stringstream ss;
        ss << "utime: " << utime.toString() << '\n';
        ss << "vx: " << vx.toString() << '\n';
        ss << "vy: " << vy.toString() << '\n';
        ss << "wz: " << wz.toString() << '\n';

        return ss.str();
    }

    std::string Twist2D::encode() const
    {
        std::string msg;
        msg.append(utime.encode());
        msg.append(vx.encode());
        msg.append(vy.encode());
        msg.append(wz.encode());
        return msg;
    }

    bool Twist2D::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a Twist2D." << std::endl;
            return false;
        }

        int len = 0;
        if (!utime.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode utime." << std::endl;
            return false;
        }
        len += utime.getMsgLen();

        if (!vx.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode vx." << std::endl;
            return false;
        }
        len += vx.getMsgLen();

        if (!vy.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode vy." << std::endl;
            return false;
        }
        len += vy.getMsgLen();

        if (!wz.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode wz." << std::endl;
            return false;
        }

        return true;
    }

} // namespace std_msgs