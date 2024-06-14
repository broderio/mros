#ifndef IMESSAGE_HPP
#define IMESSAGE_HPP

#include <iostream>
#include <string>
#include <type_traits>

#define SYNC_FLAG       0xFF
#define VERSION_FLAG    0xFE

#define HEADER_LEN      7
#define FOOTER_LEN      1
#define PKG_LEN         (HEADER_LEN + FOOTER_LEN)

enum TOPIC_ID {
    MBOT_TIMESYNC = 201, 
    MBOT_ODOMETRY = 210, 
    MBOT_ODOMETRY_RESET = 211,
    MBOT_VEL_CMD = 214,
    MBOT_IMU = 220,
    MBOT_ENCODERS = 221,
    MBOT_ENCODERS_RESET = 222,
    MBOT_MOTOR_PWM_CMD = 230,
    MBOT_MOTOR_VEL_CMD = 231,
    MBOT_MOTOR_VEL = 232,
    MBOT_MOTOR_PWM = 233,
    MBOT_VEL = 234,
    MBOT_LIDAR = 240,
};

class Parser;

struct IMessage {
    friend Parser;
public:
    virtual ~IMessage() = default;
    virtual uint16_t getMsgLen() const = 0;
    uint16_t getTopicId() const { return topicId; }
private:
    virtual std::string encode() const = 0;
    virtual void decode(const std::string& msg) = 0;
    uint16_t topicId;
};

class Parser {
public:
    template<typename MsgType>
    std::string encode(const MsgType &msg, uint16_t topicId) const {
        static_assert(!std::is_pointer<MsgType>::value, "MsgType must not be a pointer");
        static_assert(std::is_base_of<IMessage, MsgType>::value, "MsgType must inherit from IMessage");

        std::string data(PKG_LEN + msg.getMsgLen(), 0);
        data[0] = SYNC_FLAG;
        data[1] = VERSION_FLAG;
        data[2] = msg.getMsgLen() & 0xFF;
        data[3] = (msg.getMsgLen() >> 8) & 0xFF;

        char cs1_addends[2] = {data[2], data[3]};
        data[4] = checksum(cs1_addends, 2);
        data[5] = topicId & 0xFF;
        data[6] = (topicId >> 8) & 0xFF;

        std::string msg_data = msg.encode();
        std::memcpy(&data[0] + HEADER_LEN, &msg_data[0], msg.getMsgLen());

        char *cs2_addends = new char[msg.getMsgLen() + 2];
        cs2_addends[0] = data[5];
        cs2_addends[1] = data[6];
        std::memcpy(cs2_addends + 2, data.c_str() + HEADER_LEN, msg.getMsgLen());
        data.back() = checksum(cs2_addends, msg.getMsgLen() + 2);
        delete[] cs2_addends;

        return data;
    }

    template<typename MsgType>
    bool decode(const std::string& data, MsgType& msg) const {
        static_assert(!std::is_pointer<MsgType>::value, "MsgType must not be a pointer");
        static_assert(std::is_base_of<IMessage, MsgType>::value, "MsgType must inherit from IMessage");

        msg = MsgType();

        if ((uint8_t)data[0] != SYNC_FLAG) {
            std::cerr << "Sync flag not found" << std::endl;
            return false;
        }

        if ((uint8_t)data[1] != VERSION_FLAG) {
            std::cerr << "Version flag not found" << std::endl;
            return false;
        }

        char cs1_addends[2] = {data[2], data[3]};
        if (data[4] != checksum(cs1_addends, 2)) {
            std::cerr << "Checksum 1 failed" << std::endl;
            return false;
        }

        int msgLen = (uint8_t)data[2] | (uint8_t)data[3] << 8;

        char *cs2_addends = new char[msgLen + 2];
        cs2_addends[0] = data[5];
        cs2_addends[1] = data[6];
        std::memcpy(cs2_addends + 2, data.c_str() + HEADER_LEN, msgLen);
        if (data.back() != checksum(cs2_addends, msgLen + 2)) {
            std::cerr << "Checksum 2 failed" << std::endl;
            delete[] cs2_addends;   
            return false;
        }
        delete[] cs2_addends;

        std::string msg_data(data.begin() + HEADER_LEN, data.end() - FOOTER_LEN);
        msg.decode(msg_data);
        msg.topicId = (uint16_t) (data[6] << 8 | data[5]);

        return true;
    }

private:
    static char checksum(char* data, int len) {
        char sum = 0;
        for (int i = 0; i < len; i++) {
            sum += data[i];
        }
        return 255 - ( ( sum ) % 256 );
    }
};

#endif // IMESSAGE_HPP