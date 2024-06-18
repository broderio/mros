#include "messages/std_msgs/time.hpp"

Time::Time() : sec(0), nsec(0) {}

Time::Time(int32_t sec, int32_t nsec) : sec(sec), nsec(nsec) {}

Time::Time(const Time& other) : sec(other.sec), nsec(other.nsec) {}

Time& Time::operator=(const Time& other) {
    if (this == &other) {
        return *this;
    }
    sec = other.sec;
    nsec = other.nsec;
    return *this;
}

uint16_t Time::getMsgLen() const {
    return 2 * sizeof(int32_t);
}

std::string Time::toString() const {
    std::stringstream ss;
    ss << "Time:\n";
    ss << '\t' << sec << "\n";
    ss << '\t' << nsec << "\n";
    return ss.str();
}

std::string Time::encode() const {
    std::string msg;
    msg.append((char*)&sec, sizeof(int32_t));
    msg.append((char*)&nsec, sizeof(int32_t));
    return msg;
}

void Time::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a Pose2D." << std::endl;
        return;
    }

    int len = 0;
    std::memcpy(&sec, msg.data(), sizeof(sec)); len += sizeof(sec);
    std::memcpy(&nsec, msg.data() + len, sizeof(nsec));
}