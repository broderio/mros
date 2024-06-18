#include <cassert>
#include "messages/sensor_msgs/jointState.hpp"

using namespace sensor_msgs;

int main() {
    JointState jointState;
    jointState.header.seq = 1;
    jointState.header.stamp.sec = 2;
    jointState.header.stamp.nsec = 3;
    jointState.header.frame_id = "frame_id";

    jointState.name.push_back(String("name1"));
    jointState.name.push_back(String("name2"));
    jointState.name.push_back(String("name3"));

    jointState.position.push_back(1.0);
    jointState.position.push_back(2.0);
    jointState.position.push_back(3.0);

    jointState.velocity.push_back(4.0);
    jointState.velocity.push_back(5.0);
    jointState.velocity.push_back(6.0);

    jointState.effort.push_back(7.0);
    jointState.effort.push_back(8.0);
    jointState.effort.push_back(9.0);

    std::cout << "Values:\n" << jointState << std::endl;

    std::string msg = Parser::encode(jointState, TOPIC_ID::MBOT_VEL_CMD);

    JointState jointState2;
    bool res = Parser::decode(msg, jointState2);
    if (!res) return 1;

    std::cout << "\nDecoded values:\n" << jointState2 << std::endl;

    return 0;
}