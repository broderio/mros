#include <cassert>
#include "messages/msg_types/pose2d.hpp"

int main() {
    Pose2D pose;
    pose.utime = 123456789;
    pose.x = 1.0;
    pose.y = 2.0;
    pose.theta = 3.0;

    std::cout << "Values:" << std::endl;
    std::cout << "\tutime: " << pose.utime << std::endl;
    std::cout << "\tx: " << pose.x << std::endl;
    std::cout << "\ty: " << pose.y << std::endl;
    std::cout << "\ttheta: " << pose.theta << std::endl;

    std::string msg = Parser::encode(pose, TOPIC_ID::MBOT_ODOMETRY);

    Pose2D pose2;
    bool res = Parser::decode(msg, pose2);
    if (!res) return 1;

    std::cout << "Decoded values:" << std::endl;
    std::cout << "\tutime: " << pose2.utime << std::endl;
    std::cout << "\tx: " << pose2.x << std::endl;
    std::cout << "\ty: " << pose2.y << std::endl;
    std::cout << "\ttheta: " << pose2.theta << std::endl;

    return 0;
}