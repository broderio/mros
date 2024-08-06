#include "messages/std_msgs/void.hpp"

namespace std_msgs
{
    Void::Void()
    {
    }

    Void::Void(const Void &other)
    {
    }

    uint16_t Void::getMsgLen() const
    {
        return 1;
    }

    std::string Void::toString() const
    {
        return "";
    }

    std::string Void::encode() const
    {
        std::string msg;
        msg.append("\0");
        return msg;
    }

    bool Void::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a Void." << std::endl;
            return false;
        }

        if (msg[0] != '\0')
        {
            return false;
        }
        return true;
    }
}