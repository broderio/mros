#include "messages/mbot_msgs/encoders.hpp"

namespace mbot_msgs
{
    Encoders::Encoders()
    {
    }

    Encoders::Encoders(std_msgs::Int64 utime, std_msgs::Int64 ticks[3], std_msgs::Int32 delta_ticks[3], std_msgs::Int32 delta_time)
    : utime(utime), delta_time(delta_time) 
    {
        for (int i = 0; i < 3; i++)
        {
            this->ticks[i] = ticks[i];
            this->delta_ticks[i] = delta_ticks[i];
        }
    }


    Encoders::Encoders(const Encoders &other)
    : utime(other.utime), delta_time(other.delta_time) 
    {
        for (int i = 0; i < 3; i++)
        {
            this->ticks[i] = other.ticks[i];
            this->delta_ticks[i] = other.delta_ticks[i];
        }
    }


    Encoders &Encoders::operator=(const Encoders &other)
    {
        if (this == &other)
        {
            return *this;
        }

        this->utime = other.utime;
        this->delta_time = other.delta_time;
        for (int i = 0; i < 3; i++)
        {
            this->ticks[i] = other.ticks[i];
            this->delta_ticks[i] = other.delta_ticks[i];
        }
        return *this;
    }


    uint16_t Encoders::getMsgLen() const
    {
        return utime.getMsgLen() + delta_time.getMsgLen() + 3 * ticks[0].getMsgLen() + 3 * delta_ticks[0].getMsgLen();
    }


    std::string Encoders::toString() const
    {
        std::stringstream ss;
        ss << "utime: " << utime.toString() << '\n';
        ss << "ticks: [";
        for (int i = 0; i < 3; i++)
        {
            ss << ticks[i].toString();
            if (i < 2)
            {
                ss << ", ";
            }
        }
        ss << "]\n";
        ss << "delta_ticks: [";
        for (int i = 0; i < 3; i++)
        {
            ss << delta_ticks[i].toString();
            if (i < 2)
            {
                ss << ", ";
            }
        }
        ss << "]\n";
        ss << "delta_time: " << delta_time.toString();
        return ss.str();
    }


    std::string Encoders::encode() const
    {
        std::string msg;
        msg.append(utime.encode());
        for (int i = 0; i < 3; i++)
        {
            msg.append(ticks[i].encode());
        }
        for (int i = 0; i < 3; i++)
        {
            msg.append(delta_ticks[i].encode());
        }
        msg.append(delta_time.encode());
        return msg;
    }


    bool Encoders::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a Encoders." << std::endl;
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
            if (!ticks[i].decode(msg.substr(len)))
            {
                std::cerr << "Error: failed to decode ticks[" << i << "]." << std::endl;
                return false;
            }
            len += ticks[i].getMsgLen();
        }

        for (int i = 0; i < 3; i++)
        {
            if (!delta_ticks[i].decode(msg.substr(len)))
            {
                std::cerr << "Error: failed to decode delta_ticks[" << i << "]." << std::endl;
                return false;
            }
            len += delta_ticks[i].getMsgLen();
        }

        if (!delta_time.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode delta_time." << std::endl;
            return false;
        }

        return true;
    }

} // namespace std_msgs