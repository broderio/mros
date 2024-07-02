#pragma once

#include "messages/message.hpp"

namespace std_msgs
{

    class Char : public IMessage
    {
    public:
        char data;

        Char(char data);

        Char();

        Char(const Char &other);

        Char &operator=(const Char &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

    class Int8 : public IMessage
    {
    public:
        int8_t data;

        Int8(int8_t data);

        Int8();

        Int8(const Int8 &other);

        Int8 &operator=(const Int8 &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

    class Int16 : public IMessage
    {
    public:
        int16_t data;

        Int16(int16_t data);

        Int16();

        Int16(const Int16 &other);

        Int16 &operator=(const Int16 &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

    class Int32 : public IMessage
    {
    public:
        int32_t data;

        Int32(int32_t data);

        Int32();

        Int32(const Int32 &other);

        Int32 &operator=(const Int32 &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

    class Int64 : public IMessage
    {
    public:
        int64_t data;

        Int64(int64_t data);

        Int64();

        Int64(const Int64 &other);

        Int64 &operator=(const Int64 &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

    class UInt8 : public IMessage
    {
    public:
        uint8_t data;

        UInt8(uint8_t data);

        UInt8();

        UInt8(const UInt8 &other);

        UInt8 &operator=(const UInt8 &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

    class UInt16 : public IMessage
    {
    public:
        uint16_t data;

        UInt16(uint16_t data);

        UInt16();

        UInt16(const UInt16 &other);

        UInt16 &operator=(const UInt16 &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

    class UInt32 : public IMessage
    {
    public:
        uint32_t data;

        UInt32(uint32_t data);

        UInt32();

        UInt32(const UInt32 &other);

        UInt32 &operator=(const UInt32 &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

    class UInt64 : public IMessage
    {
    public:
        uint64_t data;

        UInt64(uint64_t data);

        UInt64();

        UInt64(const UInt64 &other);

        UInt64 &operator=(const UInt64 &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

} // namespace std_msgs