#ifndef VECTOR3_HPP
#define VECTOR3_HPP

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"

class Vector3 : public IMessage {
public:
    float x;
    float y;
    float z;

    Vector3() : x(0), y(0), z(0) {}

    Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

    uint16_t getMsgLen() const override {
        return 3 * sizeof(float);
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append((char*)&x, sizeof(float));
        msg.append((char*)&y, sizeof(float));
        msg.append((char*)&z, sizeof(float));
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
        std::memcpy(&z, msg.data() + len, sizeof(z));
    }
};

class Vector3Stamped : public IMessage {
public:
    Header header;
    Vector3 vector;

    uint16_t getMsgLen() const override {
        return header.getMsgLen() + vector.getMsgLen();
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append(header.encode());
        msg.append(vector.encode());
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < getMsgLen()) {
            std::cerr << "Error: message is too short to be a PointStamped." << std::endl;
            return;
        }

        header.decode(msg);
        vector.decode(msg.substr(header.getMsgLen()));
    }
};

#endif // VECTOR3_HPP