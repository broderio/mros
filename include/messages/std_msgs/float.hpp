#pragma once

#include "messages/message.hpp"

namespace std_msgs
{

    class Float32 : public IMessage
    {
    public:
        float data;

        Float32(float data);

        Float32();

        Float32(const Float32 &other);

        Float32 &operator=(const Float32 &other);

        operator float() const;

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

    class Float64 : public IMessage
    {
    public:
        double data;

        Float64(double data);

        Float64();

        Float64(const Float64 &other);

        Float64 &operator=(const Float64 &other);

        operator double() const;

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };
}