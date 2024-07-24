#include "messages/std_msgs/integer.hpp"

namespace std_msgs
{
    Char::Char() : data(0) {}

    Char::Char(char data) : data(data) {}

    Char::Char(const Char &other)
    {
        data = other.data;
    }

    Char &Char::operator=(const Char &other)
    {
        if (this == &other)
        {
            return *this;
        }
        data = other.data;
        return *this;
    }

    Char::operator char() const
    {
        return data;
    }

    uint16_t Char::getMsgLen() const
    {
        return sizeof(data);
    }

    std::string Char::toString() const
    {
        std::stringstream ss;
        ss << data;
        return ss.str();
    }

    std::string Char::encode() const
    {
        std::string msg(sizeof(data), 0);
        std::memcpy(&msg[0], &data, sizeof(data));
        return msg;
    }

    bool Char::decode(const std::string &msg)
    {
        if (msg.size() < sizeof(data))
        {
            return false;
        }
        std::memcpy(&data, &msg[0], sizeof(data));
        return true;
    }

    Int8::Int8() : data(0) {}

    Int8::Int8(int8_t data) : data(data) {}

    Int8::Int8(const Int8 &other)
    {
        data = other.data;
    }

    Int8 &Int8::operator=(const Int8 &other)
    {
        if (this == &other)
        {
            return *this;
        }
        data = other.data;
        return *this;
    }

    Int8::operator int8_t() const
    {
        return data;
    }

    uint16_t Int8::getMsgLen() const
    {
        return sizeof(data);
    }

    std::string Int8::toString() const
    {
        std::stringstream ss;
        ss << data;
        return ss.str();
    }

    std::string Int8::encode() const
    {
        std::string msg(sizeof(data), 0);
        std::memcpy(&msg[0], &data, sizeof(data));
        return msg;
    }

    bool Int8::decode(const std::string &msg)
    {
        if (msg.size() < sizeof(data))
        {
            return false;
        }
        std::memcpy(&data, &msg[0], sizeof(data));
        return true;
    }

    Int16::Int16() : data(0) {}

    Int16::Int16(int16_t data) : data(data) {}

    Int16::Int16(const Int16 &other)
    {
        data = other.data;
    }

    Int16 &Int16::operator=(const Int16 &other)
    {
        if (this == &other)
        {
            return *this;
        }
        data = other.data;
        return *this;
    }

    Int16::operator int16_t() const
    {
        return data;
    }

    uint16_t Int16::getMsgLen() const
    {
        return sizeof(data);
    }

    std::string Int16::toString() const
    {
        std::stringstream ss;
        ss << data;
        return ss.str();
    }

    std::string Int16::encode() const
    {
        std::string msg(sizeof(data), 0);
        std::memcpy(&msg[0], &data, sizeof(data));
        return msg;
    }

    bool Int16::decode(const std::string &msg)
    {
        if (msg.size() < sizeof(data))
        {
            return false;
        }
        std::memcpy(&data, &msg[0], sizeof(data));
        return true;
    }

    Int32::Int32() : data(0) {}

    Int32::Int32(int32_t data) : data(data) {}

    Int32::Int32(const Int32 &other)
    {
        data = other.data;
    }

    Int32 &Int32::operator=(const Int32 &other)
    {
        if (this == &other)
        {
            return *this;
        }
        data = other.data;
        return *this;
    }

    Int32::operator int32_t() const
    {
        return data;
    }

    uint16_t Int32::getMsgLen() const
    {
        return sizeof(data);
    }

    std::string Int32::toString() const
    {
        std::stringstream ss;
        ss << data;
        return ss.str();
    }

    std::string Int32::encode() const
    {
        std::string msg(sizeof(data), 0);
        std::memcpy(&msg[0], &data, sizeof(data));
        return msg;
    }

    bool Int32::decode(const std::string &msg)
    {
        if (msg.size() < sizeof(data))
        {
            return false;
        }
        std::memcpy(&data, &msg[0], sizeof(data));
        return true;
    }

    Int64::Int64() : data(0) {}

    Int64::Int64(int64_t data) : data(data) {}

    Int64::Int64(const Int64 &other)
    {
        data = other.data;
    }

    Int64 &Int64::operator=(const Int64 &other)
    {
        if (this == &other)
        {
            return *this;
        }
        data = other.data;
        return *this;
    }

    Int64::operator int64_t() const
    {
        return data;
    }

    uint16_t Int64::getMsgLen() const
    {
        return sizeof(data);
    }

    std::string Int64::toString() const
    {
        std::stringstream ss;
        ss << data;
        return ss.str();
    }

    std::string Int64::encode() const
    {
        std::string msg(sizeof(data), 0);
        std::memcpy(&msg[0], &data, sizeof(data));
        return msg;
    }

    bool Int64::decode(const std::string &msg)
    {
        if (msg.size() < sizeof(data))
        {
            return false;
        }
        std::memcpy(&data, &msg[0], sizeof(data));
        return true;
    }

    UInt8::UInt8() : data(0) {}

    UInt8::UInt8(uint8_t data) : data(data) {}

    UInt8::UInt8(const UInt8 &other)
    {
        data = other.data;
    }

    UInt8 &UInt8::operator=(const UInt8 &other)
    {
        if (this == &other)
        {
            return *this;
        }
        data = other.data;
        return *this;
    }

    UInt8::operator uint8_t() const
    {
        return data;
    }

    uint16_t UInt8::getMsgLen() const
    {
        return sizeof(data);
    }

    std::string UInt8::toString() const
    {
        std::stringstream ss;
        ss << data;
        return ss.str();
    }

    std::string UInt8::encode() const
    {
        std::string msg(sizeof(data), 0);
        std::memcpy(&msg[0], &data, sizeof(data));
        return msg;
    }

    bool UInt8::decode(const std::string &msg)
    {
        if (msg.size() < sizeof(data))
        {
            return false;
        }
        std::memcpy(&data, &msg[0], sizeof(data));
        return true;
    }

    UInt16::UInt16() : data(0) {}

    UInt16::UInt16(uint16_t data) : data(data) {}

    UInt16::UInt16(const UInt16 &other)
    {
        data = other.data;
    }

    UInt16 &UInt16::operator=(const UInt16 &other)
    {
        if (this == &other)
        {
            return *this;
        }
        data = other.data;
        return *this;
    }

    UInt16::operator uint16_t() const
    {
        return data;
    }

    uint16_t UInt16::getMsgLen() const
    {
        return sizeof(data);
    }

    std::string UInt16::toString() const
    {
        std::stringstream ss;
        ss << data;
        return ss.str();
    }

    std::string UInt16::encode() const
    {
        std::string msg(sizeof(data), 0);
        std::memcpy(&msg[0], &data, sizeof(data));
        return msg;
    }

    bool UInt16::decode(const std::string &msg)
    {
        if (msg.size() < sizeof(data))
        {
            return false;
        }
        std::memcpy(&data, &msg[0], sizeof(data));
        return true;
    }

    UInt32::UInt32() : data(0) {}

    UInt32::UInt32(uint32_t data) : data(data) {}

    UInt32::UInt32(const UInt32 &other)
    {
        data = other.data;
    }

    UInt32 &UInt32::operator=(const UInt32 &other)
    {
        if (this == &other)
        {
            return *this;
        }
        data = other.data;
        return *this;
    }

    UInt32::operator uint32_t() const
    {
        return data;
    }

    uint16_t UInt32::getMsgLen() const
    {
        return sizeof(data);
    }

    std::string UInt32::toString() const
    {
        std::stringstream ss;
        ss << data;
        return ss.str();
    }

    std::string UInt32::encode() const
    {
        std::string msg(sizeof(data), 0);
        std::memcpy(&msg[0], &data, sizeof(data));
        return msg;
    }

    bool UInt32::decode(const std::string &msg)
    {
        if (msg.size() < sizeof(data))
        {
            return false;
        }
        std::memcpy(&data, &msg[0], sizeof(data));
        return true;
    }

    UInt64::UInt64() : data(0) {}

    UInt64::UInt64(uint64_t data) : data(data) {}

    UInt64::UInt64(const UInt64 &other)
    {
        data = other.data;
    }

    UInt64 &UInt64::operator=(const UInt64 &other)
    {
        if (this == &other)
        {
            return *this;
        }
        data = other.data;
        return *this;
    }

    UInt64::operator uint64_t() const
    {
        return data;
    }

    uint16_t UInt64::getMsgLen() const
    {
        return sizeof(data);
    }

    std::string UInt64::toString() const
    {
        std::stringstream ss;
        ss << data;
        return ss.str();
    }

    std::string UInt64::encode() const
    {
        std::string msg(sizeof(data), 0);
        std::memcpy(&msg[0], &data, sizeof(data));
        return msg;
    }

    bool UInt64::decode(const std::string &msg)
    {
        if (msg.size() < sizeof(data))
        {
            return false;
        }
        std::memcpy(&data, &msg[0], sizeof(data));
        return true;
    }
} // namespace std_msgs