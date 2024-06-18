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

    JointState();

    JointState(const Header& header, const std::vector<String>& name, const std::vector<double>& position, const std::vector<double>& velocity, const std::vector<double>& effort);

    JointState(const JointState& other);

    JointState& operator=(const JointState& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

} // namespace sensor_msgs

#endif // JOINT_STATE_HPP