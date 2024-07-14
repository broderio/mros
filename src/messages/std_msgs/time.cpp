#include "messages/std_msgs/time.hpp"

namespace std_msgs
{

    Time::Time() : sec(0), nsec(0) {}

    Time::Time(Int32 sec, Int32 nsec) : sec(sec), nsec(nsec) {}

    Time::Time(const Time &other) : sec(other.sec), nsec(other.nsec) {}

    Time &Time::operator=(const Time &other)
    {
        if (this == &other)
        {
            return *this;
        }
        sec = other.sec;
        nsec = other.nsec;
        return *this;
    }

    uint16_t Time::getMsgLen() const
    {
        return sec.getMsgLen() + nsec.getMsgLen();
    }

    std::string Time::toString() const
    {
        std::stringstream ss;
        ss << "sec: " << sec.data;
        ss << "\nnsec: " << nsec.data;
        return ss.str();
    }

    std::string Time::encode() const
    {
        std::string msg;
        msg.append(sec.encode());
        msg.append(nsec.encode());
        return msg;
    }

    bool Time::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a Time." << std::endl;
            return false;
        }

        int len = 0;
        if (!sec.decode(msg))
        {
            std::cerr << "Error: failed to decode sec." << std::endl;
            return false;
        }
        len += sec.getMsgLen();

        if (!nsec.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode nsec." << std::endl;
            return false;
        }

        return true;
    }

} // namespace std_msgs