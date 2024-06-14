#include <cassert>
#include "messages/msg_types/lidarScan.hpp"

int main() {
    LidarScan scan;
    scan.utime = 123456789;
    scan.num_ranges = 3;
    scan.ranges = {1.0, 2.0, 3.0};
    scan.thetas = {4.0, 5.0, 6.0};
    scan.times = {7, 8, 9};
    scan.intensities = {10.0, 11.0, 12.0};

    std::cout << "Values:" << std::endl;
    std::cout << "\tutime: " << scan.utime << std::endl;
    std::cout << "\tnum_ranges: " << scan.num_ranges << std::endl;
    std::cout << "\tranges: " << scan.ranges[0] << ", " << scan.ranges[1] << ", " << scan.ranges[2] << std::endl;
    std::cout << "\tthetas: " << scan.thetas[0] << ", " << scan.thetas[1] << ", " << scan.thetas[2] << std::endl;
    std::cout << "\ttimes: " << scan.times[0] << ", " << scan.times[1] << ", " << scan.times[2] << std::endl;
    std::cout << "\tintensities: " << scan.intensities[0] << ", " << scan.intensities[1] << ", " << scan.intensities[2] << std::endl;

    std::string msg = Parser::encode(scan, TOPIC_ID::MBOT_LIDAR);

    LidarScan scan2;
    bool res = Parser::decode(msg, scan2);
    if (!res) return 1;

    std::cout << "Decoded values:" << std::endl;
    std::cout << "\tutime: " << scan2.utime << std::endl;
    std::cout << "\tnum_ranges: " << scan2.num_ranges << std::endl;
    std::cout << "\tranges: " << scan2.ranges[0] << ", " << scan2.ranges[1] << ", " << scan2.ranges[2] << std::endl;
    std::cout << "\tthetas: " << scan2.thetas[0] << ", " << scan2.thetas[1] << ", " << scan2.thetas[2] << std::endl;
    std::cout << "\ttimes: " << scan2.times[0] << ", " << scan2.times[1] << ", " << scan2.times[2] << std::endl;
    std::cout << "\tintensities: " << scan2.intensities[0] << ", " << scan2.intensities[1] << ", " << scan2.intensities[2] << std::endl;

    return 0;
}