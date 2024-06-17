#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"

class Quaternion : public IMessage {
public:
    float x;
    float y;
    float z;
    float w;

    Quaternion() : x(0), y(0), z(0), w(0) {}

    Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}

    uint16_t getMsgLen() const override {
        return 4 * sizeof(float);
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append((char*)&x, sizeof(float));
        msg.append((char*)&y, sizeof(float));
        msg.append((char*)&z, sizeof(float));
        msg.append((char*)&w, sizeof(float));
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a Point." << std::endl;
            return;
        }

        int len = 0;
        std::memcpy(&x, msg.data(), sizeof(x)); len += sizeof(x);
        std::memcpy(&y, msg.data() + len, sizeof(y)); len += sizeof(y);
        std::memcpy(&z, msg.data() + len, sizeof(z)); len += sizeof(z);
        std::memcpy(&w, msg.data() + len, sizeof(w));
    }
};

class QuaternionStamped : public IMessage {
public:
    Header header;
    Quaternion q;

    uint16_t getMsgLen() const override {
        return header.getMsgLen() + q.getMsgLen();
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append(header.encode());
        msg.append(q.encode());
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a PointStamped." << std::endl;
            return;
        }

        header.decode(msg);
        q.decode(msg.substr(q.getMsgLen()));
    }
};

#endif // QUATERNION_HPP