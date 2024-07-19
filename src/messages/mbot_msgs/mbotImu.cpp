#include "messages/mbot_msgs/mbotImu.hpp"

namespace mbot_msgs
{

    MbotImu::MbotImu()
    {
    }

    MbotImu::MbotImu(std_msgs::Int64 utime, std_msgs::Float32 gyro[3], std_msgs::Float32 accel[3], std_msgs::Float32 mag[3], std_msgs::Float32 angles_rpy[3], std_msgs::Float32 angles_quat[4], std_msgs::Float32 temp)
    : utime(utime), temp(temp) 
    {
        for (int i = 0; i < 3; i++)
        {
            this->gyro[i] = gyro[i];
            this->accel[i] = accel[i];
            this->mag[i] = mag[i];
            this->angles_rpy[i] = angles_rpy[i];
        }
        for (int i = 0; i < 4; i++)
        {
            this->angles_quat[i] = angles_quat[i];
        }
    }


    MbotImu::MbotImu(const MbotImu &other)
    : utime(other.utime), temp(other.temp) 
    {
        for (int i = 0; i < 3; i++)
        {
            this->gyro[i] = other.gyro[i];
            this->accel[i] = other.accel[i];
            this->mag[i] = other.mag[i];
            this->angles_rpy[i] = other.angles_rpy[i];
        }
        for (int i = 0; i < 4; i++)
        {
            this->angles_quat[i] = other.angles_quat[i];
        }
    }

    MbotImu &MbotImu::operator=(const MbotImu &other)
    {
        if (this == &other)
        {
            return *this;
        }

        this->utime = other.utime;
        this->temp = other.temp;
        for (int i = 0; i < 3; i++)
        {
            this->gyro[i] = other.gyro[i];
            this->accel[i] = other.accel[i];
            this->mag[i] = other.mag[i];
            this->angles_rpy[i] = other.angles_rpy[i];
        }
        for (int i = 0; i < 4; i++)
        {
            this->angles_quat[i] = other.angles_quat[i];
        }
        return *this;
    }

    uint16_t MbotImu::getMsgLen() const
    {
        return utime.getMsgLen() + temp.getMsgLen() + 3 * gyro[0].getMsgLen() + 3 * accel[0].getMsgLen() + 3 * mag[0].getMsgLen() + 3 * angles_rpy[0].getMsgLen() + 4 * angles_quat[0].getMsgLen();
    }

    std::string MbotImu::toString() const
    {
        std::stringstream ss;
        ss << "utime: " << utime.toString() << '\n';
        ss << "gyro: [";
        for (int i = 0; i < 3; i++)
        {
            ss << gyro[i].toString();
            if (i < 2)
            {
                ss << ", ";
            }
        }
        ss << "]\n";
        ss << "accel: [";
        for (int i = 0; i < 3; i++)
        {
            ss << accel[i].toString();
            if (i < 2)
            {
                ss << ", ";
            }
        }
        ss << "]\n";
        ss << "mag: [";
        for (int i = 0; i < 3; i++)
        {
            ss << mag[i].toString();
            if (i < 2)
            {
                ss << ", ";
            }
        }
        ss << "]\n";
        ss << "angles_rpy: [";
        for (int i = 0; i < 3; i++)
        {
            ss << angles_rpy[i].toString();
            if (i < 2)
            {
                ss << ", ";
            }
        }
        ss << "]\n";
        ss << "angles_quat: [";
        for (int i = 0; i < 4; i++)
        {
            ss << angles_quat[i].toString();
            if (i < 3)
            {
                ss << ", ";
            }
        }
        ss << "]\n";
        ss << "temp: " << temp.toString() << '\n';
        return ss.str();
    }

    std::string MbotImu::encode() const
    {
        std::string msg;
        msg.append(utime.encode());
        for (int i = 0; i < 3; i++)
        {
            msg.append(gyro[i].encode());
        }
        for (int i = 0; i < 3; i++)
        {
            msg.append(accel[i].encode());
        }
        for (int i = 0; i < 3; i++)
        {
            msg.append(mag[i].encode());
        }
        for (int i = 0; i < 3; i++)
        {
            msg.append(angles_rpy[i].encode());
        }
        for (int i = 0; i < 4; i++)
        {
            msg.append(angles_quat[i].encode());
        }
        msg.append(temp.encode());
        return msg;
    }

    bool MbotImu::decode(const std::string &msg)
    {
        if (msg.size() < getMsgLen())
        {
            std::cerr << "Error: message is too short to be a MbotImu." << std::endl;
            return false;
        }

        int len = 0;
        if (!utime.decode(msg))
        {
            std::cerr << "Error: failed to decode utime." << std::endl;
            return false;
        }
        len += utime.getMsgLen();

        for (int i = 0; i < 3; i++)
        {
            if (!gyro[i].decode(msg.substr(len)))
            {
                std::cerr << "Error: failed to decode gyro[" << i << "]." << std::endl;
                return false;
            }
            len += gyro[i].getMsgLen();
        }

        for (int i = 0; i < 3; i++)
        {
            if (!accel[i].decode(msg.substr(len)))
            {
                std::cerr << "Error: failed to decode accel[" << i << "]." << std::endl;
                return false;
            }
            len += accel[i].getMsgLen();
        }

        for (int i = 0; i < 3; i++)
        {
            if (!mag[i].decode(msg.substr(len)))
            {
                std::cerr << "Error: failed to decode mag[" << i << "]." << std::endl;
                return false;
            }
            len += mag[i].getMsgLen();
        }

        for (int i = 0; i < 3; i++)
        {
            if (!angles_rpy[i].decode(msg.substr(len)))
            {
                std::cerr << "Error: failed to decode angles_rpy[" << i << "]." << std::endl;
                return false;
            }
            len += angles_rpy[i].getMsgLen();
        }

        for (int i = 0; i < 4; i++)
        {
            if (!angles_quat[i].decode(msg.substr(len)))
            {
                std::cerr << "Error: failed to decode angles_quat[" << i << "]." << std::endl;
                return false;
            }
            len += angles_quat[i].getMsgLen();
        }

        if (!temp.decode(msg.substr(len)))
        {
            std::cerr << "Error: failed to decode temp." << std::endl;
            return false;
        }

        return true;
    }

} // namespace std_msgs