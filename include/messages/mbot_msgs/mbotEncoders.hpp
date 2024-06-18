#ifndef MBOT_ENCODERS_HPP
#define MBOT_ENCODERS_HPP

#include "messages/message.hpp"

class MbotEncoders : public IMessage {
public:
    int64_t utime;
    int64_t ticks[3]; // no units
    int32_t delta_ticks[3]; // no units
    int32_t delta_time; // [usec]

    MbotEncoders();

    MbotEncoders(int64_t utime, const int64_t* ticks, const int32_t* delta_ticks, int32_t delta_time);

    MbotEncoders(const MbotEncoders& other);

    MbotEncoders& operator=(const MbotEncoders& other);
    
    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;

};

#endif // MBOT_ENCODERS_HPP