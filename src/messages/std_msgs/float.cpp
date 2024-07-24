#include "messages/std_msgs/float.hpp"

namespace std_msgs
{

    Float32::Float32(float data) : data(data) {}

    Float32::Float32() : data(0) {}

    Float32::Float32(const Float32 &other)
    {
        data = other.data;
    }

    Float32 &Float32::operator=(const Float32 &other)
    {
        if (this == &other)
        {
            return *this;
        }
        data = other.data;
        return *this;
    }

    Float32::operator float() const
    {
        return data;
    }

    uint16_t Float32::getMsgLen() const
    {
        return sizeof(float);
    }

    std::string Float32::toString() const
    {
        std::stringstream ss;
        ss << data;
        return ss.str();
    }

    std::string Float32::encode() const
    {
        std::string msg(sizeof(data), 0);
        std::memcpy(&msg[0], &data, sizeof(data));
        return msg;
    }

    bool Float32::decode(const std::string &msg)
    {
        if (msg.size() < sizeof(data))
        {
            return false;
        }
        std::memcpy(&data, &msg[0], sizeof(data));
        return true;
    }

    Float64::Float64(double data) : data(data) {}

    Float64::Float64() : data(0) {}

    Float64::Float64(const Float64 &other)
    {
        data = other.data;
    }

    Float64 &Float64::operator=(const Float64 &other)
    {
        if (this == &other)
        {
            return *this;
        }
        data = other.data;
        return *this;
    }

    Float64::operator double() const
    {
        return data;
    }

    uint16_t Float64::getMsgLen() const
    {
        return sizeof(data);
    }

    std::string Float64::toString() const
    {
        std::stringstream ss;
        ss << data;
        return ss.str();
    }

    std::string Float64::encode() const
    {
        std::string msg(sizeof(data), 0);
        std::memcpy(&msg[0], &data, sizeof(data));
        return msg;
    }

    bool Float64::decode(const std::string &msg)
    {
        if (msg.size() < sizeof(double))
        {
            return false;
        }
        std::memcpy(&data, &msg[0], sizeof(data));
        return true;
    }
}