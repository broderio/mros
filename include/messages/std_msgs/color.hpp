#pragma once

#include "messages/message.hpp"
#include "messages/std_msgs/integer.hpp"
#include "messages/std_msgs/float.hpp"

namespace std_msgs
{

    class RGBA : public IMessage
    {
    public:
        UInt8 r;
        UInt8 g;
        UInt8 b;
        UInt8 a;

        RGBA(UInt8 r, UInt8 g, UInt8 b, UInt8 a);

        RGBA();

        RGBA(const RGBA &other);

        RGBA &operator=(const RGBA &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

    class HSV : public IMessage
    {
    public:
        Float32 h;
        Float32 s;
        Float32 v;
        UInt8 a;

        HSV(Float32 h, Float32 s, Float32 v, UInt8 a);

        HSV();

        HSV(const HSV &other);

        HSV &operator=(const HSV &other);

        uint16_t getMsgLen() const override;

        std::string toString() const override;

        std::string encode() const override;

        bool decode(const std::string &msg) override;
    };

    static HSV RGBAtoHSV(const RGBA &rgba);
    static RGBA HSVtoRGBA(const HSV &hsv);

} // namespace std_msgs