#ifndef LIDAR_SCAN_HPP
#define LIDAR_SCAN_HPP

#include <vector>
#include "messages/message.hpp"

class LidarScan : public IMessage
{
    friend Parser;
public:
    int64_t utime;
    int32_t num_ranges;
    std::vector<float> ranges;      // Measured range [m]
    std::vector<float> thetas;      // Measurement angle [rad]
    std::vector<int64_t> times;     // Measurement timestamp [usec]
    std::vector<float> intensities; // Measurement intensity [no units]

    LidarScan() : utime(0), num_ranges(0), ranges(), thetas(), times(), intensities() {}

    uint16_t getMsgLen() const override {
        return sizeof(int64_t) + sizeof(int32_t) + num_ranges * (sizeof(float) + sizeof(float) + sizeof(int64_t) + sizeof(float));
    }

private:
    virtual std::string encode() const override {
        std::string msg;
        msg.append((char*)&utime, sizeof(int64_t));
        msg.append((char*)&num_ranges, sizeof(int32_t));
        msg.append((char*)&ranges[0], num_ranges * sizeof(float));
        msg.append((char*)&thetas[0], num_ranges * sizeof(float));
        msg.append((char*)&times[0], num_ranges * sizeof(int64_t));
        msg.append((char*)&intensities[0], num_ranges * sizeof(float));
        return msg;
    }

    virtual void decode(const std::string& msg) override {
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
};

#endif // LIDAR_SCAN_HPP