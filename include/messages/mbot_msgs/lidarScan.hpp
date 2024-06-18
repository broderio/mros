#ifndef LIDAR_SCAN_HPP
#define LIDAR_SCAN_HPP

#include <vector>
#include "messages/message.hpp"

class LidarScan : public IMessage
{
public:
    int64_t utime;
    int32_t num_ranges;
    std::vector<float> ranges;      // Measured range [m]
    std::vector<float> thetas;      // Measurement angle [rad]
    std::vector<int64_t> times;     // Measurement timestamp [usec]
    std::vector<float> intensities; // Measurement intensity [no units]

    LidarScan();

    LidarScan(int64_t utime, int32_t num_ranges, const std::vector<float>& ranges, const std::vector<float>& thetas, const std::vector<int64_t>& times, const std::vector<float>& intensities);

    LidarScan(const LidarScan& other);

    LidarScan& operator=(const LidarScan& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

#endif // LIDAR_SCAN_HPP