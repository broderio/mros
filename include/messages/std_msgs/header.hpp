#ifndef HEADER_HPP
#define HEADER_HPP

#include "messages/message.hpp"
#include "messages/std_msgs/time.hpp"
#include "messages/std_msgs/string.hpp"

class Header : public IMessage {
public:
    uint32_t seq;
    Time stamp;
    String frame_id;

    uint16_t getMsgLen() const override {
        return stamp.getMsgLen() * sizeof(int32_t) + frame_id.getMsgLen();
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append((char*)&seq, sizeof(uint32_t));
        msg.append(stamp.encode());
        msg.append(frame_id.encode());
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a Pose2D." << std::endl;
            return;
        }

        int len = 0;
        std::memcpy(&seq, msg.data(), sizeof(seq)); len += sizeof(seq);
        stamp.decode(msg.substr(len)); len += stamp.getMsgLen();
        frame_id.decode(msg.substr(len));
    }
};

#endif // HEADER_HPP