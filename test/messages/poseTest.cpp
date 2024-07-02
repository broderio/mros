#include <cassert>
#include "messages/geometry_msgs/pose.hpp"

using namespace geometry_msgs;

int main() {
    Pose pose;
    pose.position.x = 1.0;
    pose.position.y = 2.0;
    pose.position.z = 3.0;
    pose.orientation.x = 4.0;
    pose.orientation.y = 5.0;
    pose.orientation.z = 6.0;
    pose.orientation.w = 7.0;

    std::cout << "Values:" << std::endl;
    std::cout << pose << std::endl;

    std::string msg = Parser::encode(pose, TOPIC_ID::MBOT_ODOMETRY);

    Pose pose2;
    bool res = Parser::decode(msg, pose2);
    if (!res) return 1;

    std::cout << "\nDecoded values:" << std::endl;
    std::cout << pose2 << std::endl;

    return 0;
}