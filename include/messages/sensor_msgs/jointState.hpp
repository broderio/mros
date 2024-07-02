#pragma once 

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"
#include "messages/std_msgs/float.hpp"

using namespace std_msgs;

namespace sensor_msgs {

class JointState : public IMessage {
public:
    Header header;
    std::vector<String> name;
    std::vector<Float64> position;
    std::vector<Float64> velocity;
    std::vector<Float64> effort;

    JointState();

    JointState(const std_msgs::Header& header, const std::vector<String>& name, const std::vector<Float64>& position, const std::vector<Float64>& velocity, const std::vector<Float64>& effort);

    JointState(const JointState& other);

    JointState& operator=(const JointState& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    bool decode(const std::string& msg) override;
};

} // namespace sensor_msgs