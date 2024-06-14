#ifndef MBOT_ENCODERS_HPP
#define MBOT_ENCODERS_HPP

#include "messages/message.hpp"

class MbotEncoders : public IMessage {
    friend Parser;
public:
    int64_t utime;
    int64_t ticks[3]; // no units
    int32_t delta_ticks[3]; // no units
    int32_t delta_time; // [usec]
    
    uint16_t getMsgLen() const override {
        return sizeof(int64_t) + 3 * sizeof(int64_t) + 3 * sizeof(int32_t) + sizeof(int32_t);
    }

private:
    virtual std::string encode() const override {
        std::string msg;
        msg.append((char*)&utime, sizeof(int64_t));
        msg.append((char*)ticks, 3 * sizeof(int64_t));
        msg.append((char*)delta_ticks, 3 * sizeof(int32_t));
        msg.append((char*)&delta_time, sizeof(int32_t));
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() != getMsgLen()) {
            std::cerr << "Error: Invalid message size to MBotEncoders::decode()" << std::endl;
            return;
        }

        std::memcpy(&utime, msg.data(), sizeof(utime));
        std::memcpy(&ticks, msg.data() + sizeof(utime), 3 * sizeof(int64_t));
        std::memcpy(&delta_ticks, msg.data() + sizeof(utime) + 3 * sizeof(int64_t), 3 * sizeof(int32_t));
        std::memcpy(&delta_time, msg.data() + sizeof(utime) + 3 * sizeof(int64_t) + 3 * sizeof(int32_t), sizeof(int32_t));
    }

};

#endif // MBOT_ENCODERS_HPP