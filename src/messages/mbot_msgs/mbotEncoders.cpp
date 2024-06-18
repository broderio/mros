#include "messages/mbot_msgs/mbotEncoders.hpp"

MbotEncoders::MbotEncoders() {
    utime = 0;
    ticks[0] = 0;
    ticks[1] = 0;
    ticks[2] = 0;
    delta_ticks[0] = 0;
    delta_ticks[1] = 0;
    delta_ticks[2] = 0;
    delta_time = 0;
}

MbotEncoders::MbotEncoders(int64_t utime, const int64_t* ticks, const int32_t* delta_ticks, int32_t delta_time) {
    this->utime = utime;
    this->ticks[0] = ticks[0];
    this->ticks[1] = ticks[1];
    this->ticks[2] = ticks[2];
    this->delta_ticks[0] = delta_ticks[0];
    this->delta_ticks[1] = delta_ticks[1];
    this->delta_ticks[2] = delta_ticks[2];
    this->delta_time = delta_time;
}

MbotEncoders::MbotEncoders(const MbotEncoders& other) {
    this->utime = other.utime;
    this->ticks[0] = other.ticks[0];
    this->ticks[1] = other.ticks[1];
    this->ticks[2] = other.ticks[2];
    this->delta_ticks[0] = other.delta_ticks[0];
    this->delta_ticks[1] = other.delta_ticks[1];
    this->delta_ticks[2] = other.delta_ticks[2];
    this->delta_time = other.delta_time;
}

MbotEncoders& MbotEncoders::operator=(const MbotEncoders& other) {
    this->utime = other.utime;
    this->ticks[0] = other.ticks[0];
    this->ticks[1] = other.ticks[1];
    this->ticks[2] = other.ticks[2];
    this->delta_ticks[0] = other.delta_ticks[0];
    this->delta_ticks[1] = other.delta_ticks[1];
    this->delta_ticks[2] = other.delta_ticks[2];
    this->delta_time = other.delta_time;
    return *this;
}

uint16_t MbotEncoders::getMsgLen() const {
    return sizeof(int64_t) + 3 * sizeof(int64_t) + 3 * sizeof(int32_t) + sizeof(int32_t);
}

std::string MbotEncoders::toString() const {
    std::stringstream ss;
    ss << "MbotEncoders: {";
    ss << "utime: " << utime << ", ";
    ss << "ticks: [" << ticks[0] << ", " << ticks[1] << ", " << ticks[2] << "], ";
    ss << "delta_ticks: [" << delta_ticks[0] << ", " << delta_ticks[1] << ", " << delta_ticks[2] << "], ";
    ss << "delta_time: " << delta_time;
    ss << "}";
    return ss.str();

}

std::string MbotEncoders::encode() const {
    std::string msg;
    msg.append((char*)&utime, sizeof(int64_t));
    msg.append((char*)ticks, 3 * sizeof(int64_t));
    msg.append((char*)delta_ticks, 3 * sizeof(int32_t));
    msg.append((char*)&delta_time, sizeof(int32_t));
    return msg;
}

void MbotEncoders::decode(const std::string& msg) {
    if (msg.size() != getMsgLen()) {
        std::cerr << "Error: Invalid message size to MbotEncoders::decode()" << std::endl;
        return;
    }

    std::memcpy(&utime, msg.data(), sizeof(utime));
    std::memcpy(&ticks, msg.data() + sizeof(utime), 3 * sizeof(int64_t));
    std::memcpy(&delta_ticks, msg.data() + sizeof(utime) + 3 * sizeof(int64_t), 3 * sizeof(int32_t));
    std::memcpy(&delta_time, msg.data() + sizeof(utime) + 3 * sizeof(int64_t) + 3 * sizeof(int32_t), sizeof(int32_t));
}