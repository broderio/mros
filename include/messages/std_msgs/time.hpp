#ifndef TIME_HPP
#define TIME_HPP

#include <sstream>

#include "messages/message.hpp"

class Time : public IMessage {
public:
    int32_t sec;
    int32_t nsec;

    Time();

    Time(int32_t sec, int32_t nsec);

    Time(const Time& other);

    Time& operator=(const Time& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

#endif // TIME_HPP