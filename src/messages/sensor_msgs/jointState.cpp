#include "messages/sensor_msgs/jointState.hpp"

namespace sensor_msgs {

JointState::JointState() {}

JointState::JointState(const Header& header, const std::vector<String>& name, const std::vector<double>& position, const std::vector<double>& velocity, const std::vector<double>& effort)
 : header(header), name(name), position(position), velocity(velocity), effort(effort) {}

JointState::JointState(const JointState& js)
    : header(js.header), name(js.name), position(js.position), velocity(js.velocity), effort(js.effort) {}

JointState& JointState::operator=(const JointState& js) {
    if (this == &js) {
        return *this;
    }
    header = js.header;
    name = js.name;
    position = js.position;
    velocity = js.velocity;
    effort = js.effort;
    return *this;
}

uint16_t JointState::getMsgLen() const {
    uint16_t nameLen = 0;
    for (const String& n : name) {
        nameLen += n.data.size();
    }
    return header.getMsgLen() + nameLen + position.size() * sizeof(double) + velocity.size() * sizeof(double) + effort.size() * sizeof(double);
}

std::string JointState::toString() const {
    std::string str;
    str.append(header.toString());
    str.append("\n");
    for (const String& n : name) {
        str.append(n.data);
        str.append("\n");
    }
    for (double p : position) {
        str.append(std::to_string(p));
        str.append("\n");
    }
    for (double v : velocity) {
        str.append(std::to_string(v));
        str.append("\n");
    }
    for (double e : effort) {
        str.append(std::to_string(e));
        str.append("\n");
    }
    return str;

}

std::string JointState::encode() const {
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

void JointState::decode(const std::string& msg) {
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

} // namespace sensor_msgs