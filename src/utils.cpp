#include "utils.hpp"

URI::URI() : ip(""), port(0) {}

URI::URI(const std::string &ip, int port) : ip(ip), port(port) {}

bool URI::operator<(const URI &other) const
{
    return ip < other.ip || (ip == other.ip && port < other.port);
}

bool URI::operator==(const URI &other) const
{
    return ip == other.ip && port == other.port;
}

std::string URI::toString() const
{
    return ip + ":" + std::to_string(port);
}

std::ostream &operator<<(std::ostream &os, const URI &uri)
{
    os << uri.toString();
    return os;
}

void sleep(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

std::string DateTime::toString() const
{
    std::string year = std::to_string(this->year);
    // Pad month, day, hour, minute, second with 0 if less than 10
    std::string month = this->month < 10 ? "0" + std::to_string(this->month) : std::to_string(this->month);
    std::string day = this->day < 10 ? "0" + std::to_string(this->day) : std::to_string(this->day);
    std::string hour = this->hour < 10 ? "0" + std::to_string(this->hour) : std::to_string(this->hour);
    std::string minute = this->minute < 10 ? "0" + std::to_string(this->minute) : std::to_string(this->minute);
    std::string second = this->second < 10 ? "0" + std::to_string(this->second) : std::to_string(this->second);

    return year + "-" + month + "-" + day + "_" + hour + ":" + minute + ":" + second;
}

DateTime getDateTime()
{
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_c);

    DateTime dt;
    dt.year = now_tm.tm_year + 1900;
    dt.month = now_tm.tm_mon + 1;
    dt.day = now_tm.tm_mday;
    dt.hour = now_tm.tm_hour;
    dt.minute = now_tm.tm_min;
    dt.second = now_tm.tm_sec;

    return dt;
}

int64_t getTimeMilli()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

int64_t getTimeNano()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

float degToRad(float deg)
{
    return deg * M_PI / 180.0;
}

float radToDeg(float rad)
{
    return rad * 180.0 / M_PI;
}

std::string getLocalIP()
{
    struct ifaddrs *ifAddrStruct = NULL;
    struct ifaddrs *ifa = NULL;
    void *tmpAddrPtr = NULL;

    getifaddrs(&ifAddrStruct);

    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next)
    {
        if (!ifa->ifa_addr)
        {
            continue;
        }
        if (ifa->ifa_addr->sa_family == AF_INET)
        { // check it is IP4
            // is a valid IP4 Address
            tmpAddrPtr = &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
            char addressBuffer[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
            if (strncmp(addressBuffer, "127", 3) != 0)
            { // Check it's not the loopback
                return std::string(addressBuffer);
            }
        }
    }
    if (ifAddrStruct != NULL)
        freeifaddrs(ifAddrStruct);
    return "";
}

std::string addTab(const std::string &str, int tabCount)
{
    // Insert tabCount number of tabs at the beginning of each line
    std::string tab(tabCount, '\t');
    std::string result = tab + str;
    size_t pos = 2;
    while ((pos = result.find("\n", pos)) != std::string::npos)
    {
        result.replace(pos, 1, "\n" + tab);
        pos += tabCount + 1;
    }
    return result;
}