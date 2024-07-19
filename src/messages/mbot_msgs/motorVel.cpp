#include "messages/mbot_msgs/motorVel.hpp"

namespace mbot_msgs
{
    MotorVel::MotorVel()
    {
    }

    MotorVel::MotorVel(std_msgs::Int64 utime, std_msgs::Float32 velocity[3])
    : utime(utime)
    {
        for (int i = 0; i < 3; i++)
        {
            this->velocity[i] = velocity[i];
        }
    }

    MotorVel::MotorVel(const MotorVel &other)
    : utime(other.utime) 
    {
        for (int i = 0; i < 3; i++)
        {
            this->velocity[i] = other.velocity[i];
        }
    }

    MotorVel &MotorVel::operator=(const MotorVel &other)
    {
        if (this == &other)
        {
            return *this;
        }

        this->utime = other.utime;
        for (int i = 0; i < 3; i++)
        {
            this->velocity[i] = other.velocity[i];
        }
        return *this;
    }

    uint16_t MotorVel::getMsgLen() const
    {
        return utime.getMsgLen() + 3 * velocity[0].getMsgLen();
    }

    std::string MotorVel::toString() const
    {
        std::stringstream ss;
        ss << "utime: " << utime.toString() << '\n';
        ss << "velocity: [";
        for (int i = 0; i < 3; i++)
        {
            ss << velocity[i].toString();
            if (i < 2)
            {
                ss << ", ";
            }
        }
        ss << "]";
        return ss.str();
    }

    std::string MotorVel::encode() const
    {
        std::stringstream ss;
        ss << utime.encode();
        for (int i = 0; i < 3; i++)
        {
            ss << velocity[i].encode();
        }
        return ss.str();
    }

    bool MotorVel::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a MotorVel." << std::endl;
            return false;
        }

        int len = 0;
        if (!utime.decode(msg))
        {
            std::cerr << "Error: failed to decode utime." << std::endl;
            return false;
        }
        len += utime.getMsgLen();

        for (int i = 0; i < 3; i++)
        {
            if (!velocity[i].decode(msg.substr(len)))
            {
                std::cerr << "Error: failed to decode velocity." << std::endl;
                return false;
            }
            len += velocity[i].getMsgLen();
        }

        return true;
    }
} // namespace std_msgs