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

    std::cout << "Values:" << std::endl;
    std::cout << "Header:" << std::endl;
    std::cout << "\tseq: " << jointState.header.seq << std::endl;
    std::cout << "\tsec: " << jointState.header.stamp.sec << std::endl;
    std::cout << "\tnsec: " << jointState.header.stamp.nsec << std::endl;
    std::cout << "\tframe_id: " << (std::string)jointState.header.frame_id << std::endl;
    std::cout << "Name:" << std::endl;
    for (const std::string& n : jointState.name) {
        std::cout << "\t" << n << std::endl;
    }
    std::cout << "Position:" << std::endl;
    for (double p : jointState.position) {
        std::cout << "\t" << p << std::endl;
    }
    std::cout << "Velocity:" << std::endl;
    for (double v : jointState.velocity) {
        std::cout << "\t" << v << std::endl;
    }
    std::cout << "Effort:" << std::endl;
    for (double e : jointState.effort) {
        std::cout << "\t" << e << std::endl;
    }

    std::string msg = Parser::encode(jointState, TOPIC_ID::MBOT_VEL_CMD);

    JointState jointState2;
    bool res = Parser::decode(msg, jointState2);
    if (!res) return 1;

    std::cout << "\nDecoded values:" << std::endl;
    std::cout << "Topic ID: " << jointState2.getTopicId() << std::endl;
    std::cout << "Message length: " << jointState2.getMsgLen() << std::endl;
    std::cout << "Header:" << std::endl;
    std::cout << "\tseq: " << jointState2.header.seq << std::endl;
    std::cout << "\tsec: " << jointState2.header.stamp.sec << std::endl;
    std::cout << "\tnsec: " << jointState2.header.stamp.nsec << std::endl;
    std::cout << "\tframe_id: " << (std::string)jointState2.header.frame_id << std::endl;
    std::cout << "Name:" << std::endl;
    for (const std::string& n : jointState2.name) {
        std::cout << "\t" << n << std::endl;
    }
    std::cout << "Position:" << std::endl;
    for (double p : jointState2.position) {
        std::cout << "\t" << p << std::endl;
    }
    std::cout << "Velocity:" << std::endl;
    for (double v : jointState2.velocity) {
        std::cout << "\t" << v << std::endl;
    }
    std::cout << "Effort:" << std::endl;
    for (double e : jointState2.effort) {
        std::cout << "\t" << e << std::endl;
    }

    return 0;
}