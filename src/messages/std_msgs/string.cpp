#include "messages/std_msgs/string.hpp"

namespace std_msgs
{

    String::String() : data("") {}

    String::String(const std::string &data) : data(data) {}

    String::String(const String &other) : data(other.data) {}

    String String::operator=(const std::string &data)
    {
        this->data = data;
        return *this;
    }

    uint16_t String::getMsgLen() const
    {
        return 4 + data.size();
    }

    std::string String::toString() const
    {
        std::stringstream ss;
        ss << data;
        return ss.str();
    }

    std::string String::encode() const
    {
        std::string msg(4, 0);

        uint32_t size = data.size();
        std::memcpy(&msg[0], &size, 4);

        // Encode the string
        msg.append(data);

        return msg;
    }

    bool String::decode(const std::string &msg)
    {
        // Deocde the length of the string
        uint32_t len;
        std::memcpy(&len, &msg[0], 4);

        // Decode the string
        data = msg.substr(4, len);

        return true;
    }

} // namespace std_msgs