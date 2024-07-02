#include "messages/std_msgs/color.hpp"

namespace std_msgs
{

    RGBA::RGBA() : r(0), g(0), b(0), a(0) {}

    RGBA::RGBA(UInt8 r, UInt8 g, UInt8 b, UInt8 a) : r(r), g(g), b(b), a(a) {}

    RGBA::RGBA(const RGBA &other)
    {
        r = other.r;
        g = other.g;
        b = other.b;
        a = other.a;
    }

    RGBA &RGBA::operator=(const RGBA &other)
    {
        if (this == &other)
        {
            return *this;
        }
        r = other.r;
        g = other.g;
        b = other.b;
        a = other.a;
        return *this;
    }

    uint16_t RGBA::getMsgLen() const
    {
        return r.getMsgLen() + g.getMsgLen() + b.getMsgLen() + a.getMsgLen();
    }

    std::string RGBA::toString() const
    {
        std::stringstream ss;
        ss << "r: " << r.data << '\n';
        ss << "g: " << g.data << '\n';
        ss << "b: " << b.data << '\n';
        ss << "a: " << a.data;
        return ss.str();
    }

    std::string RGBA::encode() const
    {
        std::string msg;
        msg.append(r.encode());
        msg.append(g.encode());
        msg.append(b.encode());
        msg.append(a.encode());
        return msg;
    }

    bool RGBA::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a Color." << std::endl;
            return false;
        }

        int len = 0;
        if (!r.decode(msg))
        {
            std::cerr << "Error: failed to decode r." << std::endl;
            return false;
        }
        len += r.getMsgLen();

        if (!g.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode g." << std::endl;
            return false;
        }
        len += g.getMsgLen();

        if (!b.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode b." << std::endl;
            return false;
        }
        len += b.getMsgLen();

        if (!a.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode a." << std::endl;
            return false;
        }

        return true;
    }

    HSV::HSV() : h(0), s(0), v(0), a(0) {}

    HSV::HSV(Float32 h, Float32 s, Float32 v, UInt8 a) : h(h), s(s), v(v), a(a) {}

    HSV::HSV(const HSV &other)
    {
        h = other.h;
        s = other.s;
        v = other.v;
        a = other.a;
    }

    HSV &HSV::operator=(const HSV &other)
    {
        if (this == &other)
        {
            return *this;
        }
        h = other.h;
        s = other.s;
        v = other.v;
        a = other.a;
        return *this;
    }

    uint16_t HSV::getMsgLen() const
    {
        return h.getMsgLen() + s.getMsgLen() + v.getMsgLen() + a.getMsgLen();
    }

    std::string HSV::toString() const
    {
        std::stringstream ss;
        ss << "h: " << h.data << '\n';
        ss << "s: " << s.data << '\n';
        ss << "v: " << v.data << '\n';
        ss << "a: " << a.data;
        return ss.str();
    }

    std::string HSV::encode() const
    {
        std::string msg;
        msg.append(h.encode());
        msg.append(s.encode());
        msg.append(v.encode());
        msg.append(a.encode());
        return msg;
    }

    bool HSV::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a Color." << std::endl;
            return false;
        }

        int len = 0;
        if (!h.decode(msg))
        {
            std::cerr << "Error: failed to decode h." << std::endl;
            return false;
        }
        len += h.getMsgLen();

        if (!s.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode s." << std::endl;
            return false;
        }
        len += s.getMsgLen();

        if (!v.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode v." << std::endl;
            return false;
        }
        len += v.getMsgLen();

        if (!a.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode a." << std::endl;
            return false;
        }
        
        return true;
    }

    HSV RGBAtoHSV(const RGBA &rgba)
    {
        HSV hsv;
        float r = rgba.r.data / 255;
        float g = rgba.g.data / 255;
        float b = rgba.b.data / 255;
        float cmax = std::max(r, std::max(g, b));
        float cmin = std::min(r, std::min(g, b));
        float delta = cmax - cmin;

        if (delta == 0)
        {
            hsv.h = 0;
        }
        else if (cmax == r)
        {
            hsv.h = 60 * fmod((g - b) / delta, 6);
        }
        else if (cmax == g)
        {
            hsv.h = 60 * ((b - r) / delta + 2);
        }
        else
        {
            hsv.h = 60 * ((r - g) / delta + 4);
        }

        if (cmax == 0)
        {
            hsv.s = 0;
        }
        else
        {
            hsv.s = delta / cmax;
        }

        hsv.v = cmax;
        return hsv;
    }

    RGBA HSVtoRGBA(const HSV &hsv) {
        float h = hsv.h.data;
        float s = hsv.s.data;
        float v = hsv.v.data;

        float c = v * s;
        float x = c * (1 - abs(fmod(h / 60, 2) - 1));
        float m = v - c;

        float r, g, b;
        if (h < 60)
        {
            r = c;
            g = x;
            b = 0;
        }
        else if (h < 120)
        {
            r = x;
            g = c;
            b = 0;
        }
        else if (h < 180)
        {
            r = 0;
            g = c;
            b = x;
        }
        else if (h < 240)
        {
            r = 0;
            g = x;
            b = c;
        }
        else if (h < 300)
        {
            r = x;
            g = 0;
            b = c;
        }
        else
        {
            r = c;
            g = 0;
            b = x;
        }

        UInt8 r8 = r * 255 + m;
        UInt8 g8 = g * 255 + m;
        UInt8 b8 = b * 255 + m;

        return RGBA(r8, g8, b8, hsv.a);
    }

} // namespace std_msgs