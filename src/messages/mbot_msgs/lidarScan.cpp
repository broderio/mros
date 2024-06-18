#include "messages/mbot_msgs/lidarScan.hpp"

LidarScan::LidarScan() : utime(0), num_ranges(0), ranges(), thetas(), times(), intensities() {}

LidarScan::LidarScan(int64_t utime, int32_t num_ranges, const std::vector<float>& ranges, const std::vector<float>& thetas, const std::vector<int64_t>& times, const std::vector<float>& intensities)
    : utime(utime), num_ranges(num_ranges), ranges(ranges), thetas(thetas), times(times), intensities(intensities) {}

LidarScan::LidarScan(const LidarScan& other) : utime(other.utime), num_ranges(other.num_ranges), ranges(other.ranges), thetas(other.thetas), times(other.times), intensities(other.intensities) {}

LidarScan& LidarScan::operator=(const LidarScan& other) {
    if (this == &other) {
        return *this;
    }

    utime = other.utime;
    num_ranges = other.num_ranges;
    ranges = other.ranges;
    thetas = other.thetas;
    times = other.times;
    intensities = other.intensities;

    return *this;
}

uint16_t LidarScan::getMsgLen() const {
    return sizeof(int64_t) + sizeof(int32_t) + num_ranges * (sizeof(float) + sizeof(float) + sizeof(int64_t) + sizeof(float));
}

std::string LidarScan::toString() const {
    std::string str;
    str.append("utime: " + std::to_string(utime) + "\n");
    str.append("num_ranges: " + std::to_string(num_ranges) + "\n");
    str.append("ranges: [");
    for (int i = 0; i < num_ranges; i++) {
        str.append(std::to_string(ranges[i]) + ", ");
    }
    str.append("]\n");
    str.append("thetas: [");
    for (int i = 0; i < num_ranges; i++) {
        str.append(std::to_string(thetas[i]) + ", ");
    }
    str.append("]\n");
    str.append("times: [");
    for (int i = 0; i < num_ranges; i++) {
        str.append(std::to_string(times[i]) + ", ");
    }
    str.append("]\n");
    str.append("intensities: [");
    for (int i = 0; i < num_ranges; i++) {
        str.append(std::to_string(intensities[i]) + ", ");
    }
    str.append("]\n");
    return str;

}

std::string LidarScan::encode() const {
    std::string msg;
    msg.append((char*)&utime, sizeof(int64_t));
    msg.append((char*)&num_ranges, sizeof(int32_t));
    msg.append((char*)&ranges[0], num_ranges * sizeof(float));
    msg.append((char*)&thetas[0], num_ranges * sizeof(float));
    msg.append((char*)&times[0], num_ranges * sizeof(int64_t));
    msg.append((char*)&intensities[0], num_ranges * sizeof(float));
    return msg;
}

void LidarScan::decode(const std::string& msg) {
    if (msg.size() < sizeof(int64_t) + sizeof(int32_t)) {
        std::cerr << "Error: message is too short to be a LidarScan." << std::endl;
        return;
    }

    std::memcpy(&utime, msg.data(), sizeof(utime));
    std::memcpy(&num_ranges, msg.data() + sizeof(utime), sizeof(num_ranges));

    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a LidarScan." << std::endl;
        return;
    }

    ranges.resize(num_ranges);
    thetas.resize(num_ranges);
    times.resize(num_ranges);
    intensities.resize(num_ranges);

    std::memcpy(&ranges[0], msg.data() + sizeof(utime) + sizeof(num_ranges), num_ranges * sizeof(float));
    std::memcpy(&thetas[0], msg.data() + sizeof(utime) + sizeof(num_ranges) + num_ranges * sizeof(float), num_ranges * sizeof(float));
    std::memcpy(&times[0], msg.data() + sizeof(utime) + sizeof(num_ranges) + num_ranges * sizeof(float) + num_ranges * sizeof(float), num_ranges * sizeof(int64_t));
    std::memcpy(&intensities[0], msg.data() + sizeof(utime) + sizeof(num_ranges) + num_ranges * sizeof(float) + num_ranges * sizeof(float) + num_ranges * sizeof(int64_t), num_ranges * sizeof(float));
}