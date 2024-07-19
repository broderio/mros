#include "messages/mbot_msgs/motorPwm.hpp"

namespace mbot_msgs
{
    MotorPwm::MotorPwm()
    {
    }

    MotorPwm::MotorPwm(std_msgs::Int64 utime, std_msgs::Float32 pwm[3])
    : utime(utime) 
    {
        for (int i = 0; i < 3; i++)
        {
            this->pwm[i] = pwm[i];
        }
    }

    MotorPwm::MotorPwm(const MotorPwm &other)
    : utime(other.utime) 
    {
        for (int i = 0; i < 3; i++)
        {
            this->pwm[i] = other.pwm[i];
        }
    }

    MotorPwm &MotorPwm::operator=(const MotorPwm &other)
    {
        if (this == &other)
        {
            return *this;
        }

        this->utime = other.utime;
        for (int i = 0; i < 3; i++)
        {
            this->pwm[i] = other.pwm[i];
        }
        return *this;
    }

    uint16_t MotorPwm::getMsgLen() const
    {
        return utime.getMsgLen() + 3 * pwm[0].getMsgLen();
    }

    std::string MotorPwm::toString() const
    {
        std::stringstream ss;
        ss << "utime: " << utime.toString() << '\n';
        ss << "pwm: [";
        for (int i = 0; i < 3; i++)
        {
            ss << pwm[i].toString();
            if (i < 2)
            {
                ss << ", ";
            }
        }
        ss << "]";
        return ss.str();
    }

    std::string MotorPwm::encode() const
    {
        std::string msg;
        msg.append(utime.encode());
        for (int i = 0; i < 3; i++)
        {
            msg.append(pwm[i].encode());
        }
        return msg;
    }

    bool MotorPwm::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a MotorPwm." << std::endl;
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
            if (!pwm[i].decode(msg.substr(len)))
            {
                std::cerr << "Error: failed to decode pwm[" << i << "]." << std::endl;
                return false;
            }
            len += pwm[i].getMsgLen();
        }

        return true;
    }

} // namespace std_msgs