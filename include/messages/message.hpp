#pragma once

#include <iostream>
#include <ostream>
#include <vector>
#include <string>
#include <type_traits>
#include <sstream>

#include "utils.hpp"

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

/**
 * @brief The base interface for all message types.
 * 
 * This interface defines the common functionality that all message types should implement.
 */
class IMessage {
public:

    IMessage() = default;

    IMessage(const IMessage& other) = delete;

    virtual ~IMessage() = default;

    virtual uint16_t getMsgLen() const = 0;

    uint16_t getTopicId() const { return topicId; }

    virtual std::string toString() const = 0;

    virtual std::string encode() const = 0;

    virtual bool decode(const std::string& msg) = 0;

    uint16_t topicId;
    
protected:

    template<typename MsgType>
    static std::string encodeVector(const std::vector<MsgType>& vec) {
        static_assert(!std::is_pointer<MsgType>::value, "MsgType must not be a pointer");
        static_assert(std::is_base_of<IMessage, MsgType>::value, "MsgType must inherit from IMessage");

        std::string msg(4, 0);

        // Add the number of elements in the vector (4 bytes)
        uint32_t size = vec.size();
        std::memcpy(&msg[0], &size, 4);

        // Add the encoded messages
        for (const MsgType& m : vec) {
            msg.append(m.encode());
        }

        return msg;
    }

    template<typename MsgType>
    static bool decodeVector(std::vector<MsgType>& vec, const std::string& msg) {
        static_assert(!std::is_pointer<MsgType>::value, "MsgType must not be a pointer");
        static_assert(std::is_base_of<IMessage, MsgType>::value, "MsgType must inherit from IMessage");

        uint32_t size;
        std::memcpy(&size, &msg[0], 4);

        // Decode the messages
        int len = 4;
        for (int i = 0; i < size; i++) {
            MsgType m;
            if (!m.decode(msg.substr(len))) {
                return false;
            }
            vec.push_back(m);
            len += m.getMsgLen();
        }

        return true;
    }

    template<typename MsgType>
    static uint32_t getVectorLen(const std::vector<MsgType>& vec) {
        static_assert(!std::is_pointer<MsgType>::value, "MsgType must not be a pointer");
        static_assert(std::is_base_of<IMessage, MsgType>::value, "MsgType must inherit from IMessage");

        uint32_t len = 4;
        for (const MsgType& m : vec) {
            len += m.getMsgLen();
        }

        return len;
    }
};

std::ostream& operator<<(std::ostream& os, const IMessage& msg);

class Parser {
public:

    template<typename MsgType>
    static std::string encode(const MsgType &msg, uint16_t topicId) {
        static_assert(!std::is_pointer<MsgType>::value, "MsgType must not be a pointer");
        static_assert(std::is_base_of<IMessage, MsgType>::value, "MsgType must inherit from IMessage");

        // Create the header of the message
        std::string data(PKG_LEN + msg.getMsgLen(), 0);
        data[0] = SYNC_FLAG;
        data[1] = VERSION_FLAG;
        uint16_t msgLen = msg.getMsgLen();
        data[2] = msgLen & 0xFF;
        data[3] = (msgLen >> 8) & 0xFF;

        // Calculate the checksum of the header
        char cs1_addends[2] = {data[2], data[3]};
        data[4] = checksum(cs1_addends, 2);
        data[5] = topicId & 0xFF;
        data[6] = (topicId >> 8) & 0xFF;

        // Encode the message
        std::string msg_data = msg.encode();
        
        // Copy the message data into the byte string
        std::memcpy(&data[0] + HEADER_LEN, &msg_data[0], msg.getMsgLen());

        // Calculate the checksum of the message
        char *cs2_addends = new char[msg.getMsgLen() + 2];
        cs2_addends[0] = data[5];
        cs2_addends[1] = data[6];
        std::memcpy(cs2_addends + 2, data.c_str() + HEADER_LEN, msg.getMsgLen());

        // Add the checksum to the end of the message
        data.back() = checksum(cs2_addends, msg.getMsgLen() + 2);

        // Clean up
        delete[] cs2_addends;

        return data;
    }

    template<typename MsgType>
    static bool decode(const std::string& data, MsgType& msg) {
        static_assert(!std::is_pointer<MsgType>::value, "MsgType must not be a pointer");
        static_assert(std::is_base_of<IMessage, MsgType>::value, "MsgType must inherit from IMessage");

        // Clear the message object
        msg = MsgType();

        // Check the sync flag
        if ((uint8_t)data[0] != SYNC_FLAG) {
            std::cerr << "Sync flag not found" << std::endl;
            return false;
        }

        // Check the version flag
        if ((uint8_t)data[1] != VERSION_FLAG) {
            std::cerr << "Version flag not found" << std::endl;
            return false;
        }

        // Check the checksum of the header
        char cs1_addends[2] = {data[2], data[3]};
        if (data[4] != checksum(cs1_addends, 2)) {
            std::cerr << "Checksum 1 failed" << std::endl;
            return false;
        }

        // Get the message length
        int msgLen = (uint8_t)data[2] | (uint8_t)data[3] << 8;

        // Check the checksum of the message
        char *cs2_addends = new char[msgLen + 2];
        cs2_addends[0] = data[5];
        cs2_addends[1] = data[6];
        std::memcpy(cs2_addends + 2, data.c_str() + HEADER_LEN, msgLen);
        if (data.back() != checksum(cs2_addends, msgLen + 2)) {
            std::cerr << "Checksum 2 failed" << std::endl;
            delete[] cs2_addends;   
            return false;
        }

        // Clean up
        delete[] cs2_addends;

        // Decode the message
        std::string msg_data(data.begin() + HEADER_LEN, data.end() - FOOTER_LEN);
        if (!msg.decode(msg_data)) {
            std::cerr << "Failed to decode message" << std::endl;
            return false;
        }

        // Set the topic ID
        msg.topicId = (uint16_t) (data[6] << 8 | (data[5] & 0xFF));

        return true;
    }

    static bool getTopicID(const std::string& data, uint16_t& topicId) {
         // Check the sync flag
        if ((uint8_t)data[0] != SYNC_FLAG) {
            std::cerr << "Sync flag not found" << std::endl;
            return false;
        }

        // Check the version flag
        if ((uint8_t)data[1] != VERSION_FLAG) {
            std::cerr << "Version flag not found" << std::endl;
            return false;
        }

        // Check the checksum of the header
        char cs1_addends[2] = {data[2], data[3]};
        if (data[4] != checksum(cs1_addends, 2)) {
            std::cerr << "Checksum 1 failed" << std::endl;
            return false;
        }

        topicId = (uint16_t) (data[6] << 8 | (data[5] & 0xFF));
        return true;
    }

private:

    static char checksum(char* data, int len);

};