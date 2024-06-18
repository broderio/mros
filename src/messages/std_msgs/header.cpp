#include "messages/std_msgs/header.hpp"

Header::Header() : seq(0), stamp(), frame_id() {}

Header::Header(uint32_t seq, const Time& stamp, const String& frame_id) : seq(seq), stamp(stamp), frame_id(frame_id) {}

Header::Header(const Header& other) : seq(other.seq), stamp(other.stamp), frame_id(other.frame_id) {}

Header& Header::operator=(const Header& other) {
    if (this == &other) {
        return *this;
    }
    seq = other.seq;
    stamp = other.stamp;
    frame_id = other.frame_id;
    return *this;
}

uint16_t Header::getMsgLen() const {
    return  sizeof(int32_t) + stamp.getMsgLen() + frame_id.getMsgLen();
}

std::string Header::toString() const {
    std::stringstream ss;
    ss << "Header:\n";
    ss << '\t' << seq << "\n";
    ss << stamp.toString();
    ss << frame_id.toString();
    return ss.str();
}

std::string Header::encode() const {
    std::string msg;
    msg.append((char*)&seq, sizeof(uint32_t));
    msg.append(stamp.encode());
    msg.append(frame_id.encode());
    return msg;
}

void Header::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Pose2D." << std::endl;
        return;
    }

    int len = 0;
    std::memcpy(&seq, msg.data(), sizeof(seq)); len += sizeof(seq);
    stamp.decode(msg.substr(len)); len += stamp.getMsgLen();
    frame_id.decode(msg.substr(len, msg.find_first_of('\0', len) - len));
}