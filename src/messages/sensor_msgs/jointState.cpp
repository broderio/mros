#include "messages/sensor_msgs/jointState.hpp"

namespace sensor_msgs
{

    JointState::JointState() {}

    JointState::JointState(const Header &header, const std::vector<String> &name, const std::vector<Float64> &position, const std::vector<Float64> &velocity, const std::vector<Float64> &effort)
        : header(header), name(name), position(position), velocity(velocity), effort(effort) {}

    JointState::JointState(const JointState &js)
    {
        header = js.header;
        name = js.name;
        position = js.position;
        velocity = js.velocity;
        effort = js.effort;
    }

    JointState &JointState::operator=(const JointState &js)
    {
        if (this == &js)
        {
            return *this;
        }
        header = js.header;
        name = js.name;
        position = js.position;
        velocity = js.velocity;
        effort = js.effort;
        return *this;
    }

    uint16_t JointState::getMsgLen() const
    {
        return header.getMsgLen() + IMessage::getVectorLen(name) + IMessage::getVectorLen(position) + IMessage::getVectorLen(velocity) + IMessage::getVectorLen(effort);
    }

    std::string JointState::toString() const
    {
        std::stringstream ss;
        ss << "header:\n" << addTab(header.toString(), 1) << std::endl;
        ss << "name: [\n";
        for (size_t i = 0; i < name.size(); i++)
        {
            ss << addTab(name[i].toString(), 1) << ",\n";
        }
        ss << "]\n";
        ss << "position: [\n";
        for (size_t i = 0; i < position.size(); i++)
        {
            ss << addTab(position[i].toString(), 1) << ",\n";
        }
        ss << "]\n";
        ss << "velocity: [\n";
        for (size_t i = 0; i < velocity.size(); i++)
        {
            ss << addTab(velocity[i].toString(), 1) << ",\n";
        }
        ss << "]\n";
        ss << "effort: [\n";
        for (size_t i = 0; i < effort.size(); i++)
        {
            ss << addTab(effort[i].toString(), 1) << ",\n";
        }
        ss << "]\n";
        return ss.str();
    }

    std::string JointState::encode() const
    {
        std::string msg;
        msg.append(header.encode());
        msg.append(IMessage::encodeVector(name));
        msg.append(IMessage::encodeVector(position));
        msg.append(IMessage::encodeVector(velocity));
        msg.append(IMessage::encodeVector(effort));
        return msg;
    }

    bool JointState::decode(const std::string &msg)
    {
        if (msg.size() < header.getMsgLen())
        {
            std::cerr << "Error: message is too short to be a JointState." << std::endl;
            return false;
        }

        int len = 0;
        if (!header.decode(msg))
        {
            std::cerr << "Error: failed to decode header." << std::endl;
            return false;
        }
        len += header.getMsgLen();
        
        if (!IMessage::decodeVector(name, msg.substr(len)))
        {
            std::cerr << "Error: failed to decode name vector." << std::endl;
            return false;
        }
        len += IMessage::getVectorLen(name);

        if (!IMessage::decodeVector(position, msg.substr(len)))
        {
            std::cerr << "Error: failed to decode position vector." << std::endl;
            return false;
        }
        len += IMessage::getVectorLen(position);

        if (!IMessage::decodeVector(velocity, msg.substr(len)))
        {
            std::cerr << "Error: failed to decode velocity vector." << std::endl;
            return false;
        }
        len += IMessage::getVectorLen(velocity);

        if (!IMessage::decodeVector(effort, msg.substr(len)))
        {
            std::cerr << "Error: failed to decode effort vector." << std::endl;
            return false;
        }

        return true;
    }

} // namespace sensor_msgs