#include "messages/mbot_msgs/timestamp.hpp"

namespace mbot_msgs
{
    Timestamp::Timestamp()
    {
    }

    Timestamp::Timestamp(std_msgs::Int64 utime)
    : utime(utime) {}

    Timestamp::Timestamp(const Timestamp &other)
    : utime(other.utime) {}

    Timestamp &Timestamp::operator=(const Timestamp &other)
    {
        if (this == &other)
        {
            return *this;
        }

        this->utime = other.utime;
        return *this;
    }

    uint16_t Timestamp::getMsgLen() const
    {
        return utime.getMsgLen();
    }

    std::string Timestamp::toString() const
    {
        std::stringstream ss;
        ss << "utime: " << utime.toString() << '\n';
        return ss.str();
    }

    std::string Timestamp::encode() const
    {
        std::string msg;
        msg.append(utime.encode());
        return msg;
    }

    bool Timestamp::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a Timestamp." << std::endl;
            return false;
        }

        int len = 0;
        if (!utime.decode(msg))
        {
            std::cerr << "Error: failed to decode utime." << std::endl;
            return false;
        }

        return true;
    }

} // namespace std_msgs