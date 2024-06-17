#ifndef JOINT_STATE_HPP
#define JOINT_STATE_HPP

#include <vector>
#include <string>

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"

namespace sensor_msgs {

class JointState : public IMessage {
public:
    Header header;
    std::vector<String> name;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;

    JointState() {}

    uint16_t getMsgLen() const override {
        uint16_t nameLen = 0;
        for (const std::string& n : name) {
            nameLen += n.size();
        }
        return header.getMsgLen() + nameLen + position.size() * sizeof(double) + velocity.size() * sizeof(double) + effort.size() * sizeof(double);
    }

    virtual std::string encode() const override {
        std::string msg;
        msg.append(header.encode());
        for (const String& n : name) {
            msg.append(n.encode());
        }
        msg.append((char*)position.data(), position.size() * sizeof(double));
        msg.append((char*)velocity.data(), velocity.size() * sizeof(double));
        msg.append((char*)effort.data(), effort.size() * sizeof(double));
        return msg;
    }

    virtual void decode(const std::string& msg) override {
        if (msg.size() < header.getMsgLen()) {
            std::cerr << "Error: message is too short to be a JointState." << std::endl;
            return;
        }

        int len = 0;
        header.decode(msg); len += header.getMsgLen();

        int count = 0;
        while (len + count * 3 * sizeof(double) < msg.size() - 1) {
            size_t nameEnd = msg.find_first_of('\0', len);
            std::string n = msg.substr(len, nameEnd - len);
            name.push_back(n);
            len = nameEnd + 1;
            count++;
        }

        position.resize(count);
        std::memcpy(position.data(), msg.data() + len, count * sizeof(double));
        len += count * sizeof(double);

        velocity.resize(count);
        std::memcpy(velocity.data(), msg.data() + len, count * sizeof(double));
        len += count * sizeof(double);

        effort.resize(count);
        std::memcpy(effort.data(), msg.data() + len, count * sizeof(double));
    }
};

} // namespace sensor_msgs

#endif // JOINT_STATE_HPP