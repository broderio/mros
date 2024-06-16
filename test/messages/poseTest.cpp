#include <cassert>
#include "messages/geometry_msgs/pose.hpp"

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
    std::cout << "Position:" << std::endl;
    std::cout << "\tx: " << pose.position.x << std::endl;
    std::cout << "\ty: " << pose.position.y << std::endl;
    std::cout << "\tz: " << pose.position.z << std::endl;
    std::cout << "Orientation:" << std::endl;
    std::cout << "\tx: " << pose.orientation.x << std::endl;
    std::cout << "\ty: " << pose.orientation.y << std::endl;
    std::cout << "\tz: " << pose.orientation.z << std::endl;
    std::cout << "\tw: " << pose.orientation.w << std::endl;

    std::string msg = Parser::encode(pose, TOPIC_ID::MBOT_ODOMETRY);

    Pose pose2;
    bool res = Parser::decode(msg, pose2);
    if (!res) return 1;

    std::cout << "\nDecoded values:" << std::endl;
    std::cout << "Topic ID: " << pose2.getTopicId() << std::endl;
    std::cout << "Message length: " << pose2.getMsgLen() << std::endl;
    std::cout << "Position:" << std::endl;
    std::cout << "\tx: " << pose2.position.x << std::endl;
    std::cout << "\ty: " << pose2.position.y << std::endl;
    std::cout << "\tz: " << pose2.position.z << std::endl;
    std::cout << "Orientation:" << std::endl;
    std::cout << "\tx: " << pose2.orientation.x << std::endl;
    std::cout << "\ty: " << pose2.orientation.y << std::endl;
    std::cout << "\tz: " << pose2.orientation.z << std::endl;
    std::cout << "\tw: " << pose2.orientation.w << std::endl;

    return 0;
}