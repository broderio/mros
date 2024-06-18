#ifndef IMESSAGE_HPP
#define IMESSAGE_HPP

#include <iostream>
#include <ostream>
#include <string>
#include <type_traits>
#include <sstream>

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
struct IMessage {
    /**
     * @brief Default constructor for the IMessage interface.
     */
    IMessage() = default;

    /**
     * @brief Copy constructor for the IMessage interface.
     * 
     * @param other The other IMessage object to copy.
     */
    IMessage(const IMessage& other) = delete;

    /**
     * @brief Destructor for the IMessage interface.
     */
    virtual ~IMessage() = default;

    /**
     * @brief Get the length of the message.
     * 
     * @return The length of the message in bytes.
     */
    virtual uint16_t getMsgLen() const = 0;

    /**
     * @brief Get the topic ID of the message.
     * 
     * @return The topic ID of the message.
     */
    uint16_t getTopicId() const { return topicId; }

    virtual std::string toString() const = 0;

    /**
     * @brief Encode the message into a byte string.
     * 
     * @return The encoded message as a string.
     */
    virtual std::string encode() const = 0;

    /**
     * @brief Decode the message from a string representation. 
     * Stores the data in the message object.
     * 
     * @param msg The byte string representation of the message.
     */
    virtual void decode(const std::string& msg) = 0;

    /**
     * @brief The topic ID of the message.
    */
    uint16_t topicId;
};

std::ostream& operator<<(std::ostream& os, const IMessage& msg);

class Parser {
public:
    /**
     * @brief Encode a message into a byte string.
     * 
     * @tparam MsgType The type of the message to encode.
     * @param msg The message to encode.
     * @param topicId The topic ID of the message.
     * @return The encoded message as a byte string in the rosserial packet format: http://wiki.ros.org/rosserial/Overview/Protocol.
     * 
    */
    template<typename MsgType>
    static std::string encode(const MsgType &msg, uint16_t topicId) {
        static_assert(!std::is_pointer<MsgType>::value, "MsgType must not be a pointer");
        static_assert(std::is_base_of<IMessage, MsgType>::value, "MsgType must inherit from IMessage");

        // Create the header of the message
        std::string data(PKG_LEN + msg.getMsgLen(), 0);
        data[0] = SYNC_FLAG;
        data[1] = VERSION_FLAG;
        data[2] = msg.getMsgLen() & 0xFF;
        data[3] = (msg.getMsgLen() >> 8) & 0xFF;

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

    /**
     * @brief Decode a message from a byte string.
     * 
     * @tparam MsgType The type of the message to decode.
     * @param data The byte string representation of the message in rosserial packet format: http://wiki.ros.org/rosserial/Overview/Protocol.
     * @param msg The message object to store the decoded data.
     * @return True if the decoding was successful, false otherwise.
     */
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
        msg.decode(msg_data);

        // Set the topic ID
        msg.topicId = (uint16_t) (data[6] << 8 | (data[5] & 0xFF));

        return true;
    }

private:

    /**
     * @brief Calculate the checksum of a byte array defined in http://wiki.ros.org/rosserial/Overview/Protocol.
     * 
     * @param data The byte array to calculate the checksum for.
     * @param len The length of the byte array.
     * @return The checksum of the byte array.
     */
    static char checksum(char* data, int len) {
        char sum = 0;
        for (int i = 0; i < len; i++) {
            sum += data[i];
        }
        return 255 - ( ( sum ) % 256 );
    }   
};

#endif // IMESSAGE_HPP