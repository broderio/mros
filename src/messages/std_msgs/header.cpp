#include "messages/std_msgs/header.hpp"

namespace std_msgs
{

    Header::Header() : seq(0), stamp(), frame_id() {}

    Header::Header(UInt32 seq, const Time &stamp, const String &frame_id) : seq(seq), stamp(stamp), frame_id(frame_id) {}

    Header::Header(const Header &other) : seq(other.seq), stamp(other.stamp), frame_id(other.frame_id) {}

    Header &Header::operator=(const Header &other)
    {
        if (this == &other)
        {
            return *this;
        }
        seq = other.seq;
        stamp = other.stamp;
        frame_id = other.frame_id;
        return *this;
    }

    uint16_t Header::getMsgLen() const
    {
        return seq.getMsgLen() + stamp.getMsgLen() + frame_id.getMsgLen();
    }

    std::string Header::toString() const
    {
        std::stringstream ss;
        ss << "seq: " << seq.data << '\n';
        ss << "stamp:\n" << addTab(stamp.toString(), 1) << '\n';
        ss << "frame_id: " << frame_id.data;
        return ss.str();
    }

    std::string Header::encode() const
    {
        std::string msg;
        msg.append(seq.encode());
        msg.append(stamp.encode());
        msg.append(frame_id.encode());
        return msg;
    }

    bool Header::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a Header." << std::endl;
            return false;
        }

        int len = 0;
        if (!seq.decode(msg))
        {
            std::cerr << "Error: failed to decode seq." << std::endl;
            return false;
        }
        len += seq.getMsgLen();

        if (!stamp.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode stamp." << std::endl;
            return false;
        }
        len += stamp.getMsgLen();

        if (!frame_id.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode frame_id." << std::endl;
            return false;
        }

        return true;
    }

} // namespace std_msgs