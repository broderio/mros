#include <cassert>
#include "messages/mbot_msgs/mbotEncoders.hpp"

int main() {
    MbotEncoders enc;
    enc.utime = 123456789;
    enc.ticks[0] = 1;
    enc.ticks[1] = 2;
    enc.ticks[2] = 3;
    enc.delta_ticks[0] = 4;
    enc.delta_ticks[1] = 5;
    enc.delta_ticks[2] = 6;
    enc.delta_time = 7;
    
    std::cout << "Values:" << std::endl;
    std::cout << "\tutime: " << enc.utime << std::endl;
    std::cout << "\tticks: " << enc.ticks[0] << ", " << enc.ticks[1] << ", " << enc.ticks[2] << std::endl;
    std::cout << "\tdelta_ticks: "<< enc.delta_ticks[0] << ", " << enc.delta_ticks[1] << ", " << enc.delta_ticks[2] << std::endl;
    std::cout << "\tdelta_time: " << enc.delta_time << std::endl;

    std::string msg = Parser::encode(enc, TOPIC_ID::MBOT_ENCODERS);

    MbotEncoders enc2;
    bool res = Parser::decode(msg, enc2);
    if (!res) return 1;

    std::cout << "Values:" << std::endl;
    std::cout << "\tutime: " << enc.utime << std::endl;
    std::cout << "\tticks: " << enc2.ticks[0] << ", " << enc2.ticks[1] << ", " << enc2.ticks[2] << std::endl;
    std::cout << "\tdelta_ticks: "<< enc2.delta_ticks[0] << ", " << enc2.delta_ticks[1] << ", " << enc2.delta_ticks[2] << std::endl;
    std::cout << "\tdelta_time: " << enc2.delta_time << std::endl;

    return 0;
}