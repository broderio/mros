#include <cassert>
#include "messages/mbot_msgs/twist2d.hpp"

int main() {
    Twist2D twist;
    twist.utime = 123456789;
    twist.vx = 1.0;
    twist.vy = 2.0;
    twist.wz = 3.0;

    std::cout << "Values:" << std::endl;
    std::cout << "\tutime: " << twist.utime << std::endl;
    std::cout << "\tvx: " << twist.vx << std::endl;
    std::cout << "\tvy: " << twist.vy << std::endl;
    std::cout << "\twz: " << twist.wz << std::endl;

    std::string msg = Parser::encode(twist, TOPIC_ID::MBOT_VEL_CMD);

    Twist2D twist2;
    bool res = Parser::decode(msg, twist2);
    if (!res) return 1;

    std::cout << "Decoded values:" << std::endl;
    std::cout << "\tutime: " << twist2.utime << std::endl;
    std::cout << "\tvx: " << twist2.vx << std::endl;
    std::cout << "\tvy: " << twist2.vy << std::endl;
    std::cout << "\twz: " << twist2.wz << std::endl;
    return 0;
}