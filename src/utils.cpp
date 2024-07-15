#include "utils.hpp"

URI::URI() : ip(""), port(0) {}

URI::URI(const std::string& ip, int port) : ip(ip), port(port) {}

bool URI::operator<(const URI& other) const {
    return ip < other.ip || (ip == other.ip && port < other.port);
}

bool URI::operator==(const URI& other) const {
    return ip == other.ip && port == other.port;
}

std::string URI::toString() const {
    return ip + ":" + std::to_string(port);
}

std::ostream& operator<<(std::ostream& os, const URI& uri) {
    os << uri.toString();
    return os;
}

void sleep(int milliseconds) {
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

std::string DateTime::toString() const {
    std::string year = std::to_string(this->year);
    // Pad month, day, hour, minute, second with 0 if less than 10
    std::string month = this->month < 10 ? "0" + std::to_string(this->month) : std::to_string(this->month);
    std::string day = this->day < 10 ? "0" + std::to_string(this->day) : std::to_string(this->day);
    std::string hour = this->hour < 10 ? "0" + std::to_string(this->hour) : std::to_string(this->hour);
    std::string minute = this->minute < 10 ? "0" + std::to_string(this->minute) : std::to_string(this->minute);
    std::string second = this->second < 10 ? "0" + std::to_string(this->second) : std::to_string(this->second);

    return year + "-" + month + "-" + day + "_" + hour + ":" + minute + ":" + second;
}

DateTime getDateTime() {
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

int64_t getTimeMilli() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

int64_t getTimeNano() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

float degToRad(float deg) {
    return deg * M_PI / 180.0;
}

float radToDeg(float rad) {
    return rad * 180.0 / M_PI;
}

std::vector<std::string> getLocalIPv4Addresses() {
    struct addrinfo hints, *info, *p;
    int gai_result;
    char hostname[1024];
    char ipstr[INET_ADDRSTRLEN];

    hostname[1023] = '\0';

    /*
       These system calls are used to access or to change the system
       hostname.  More precisely, they operate on the hostname
       associated with the calling process's UTS namespace.
       - https://man7.org/linux/man-pages/man2/gethostname.2.html
    */
    gethostname(hostname, 1023);

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET; // Use AF_INET to force IPv4
    hints.ai_socktype = SOCK_STREAM;

    /*
       getaddrinfo() returns one or more addrinfo structures as a linked 
       list (in the info struct), each of which contains an Internet 
       address that can be specified in a call to bind(2) or connect(2).
       - https://man7.org/linux/man-pages/man3/getaddrinfo.3.html
    */
    if ((gai_result = getaddrinfo(hostname, NULL, &hints, &info)) != 0) {
        throw std::runtime_error(gai_strerror(gai_result));
    }

    std::vector<std::string> ipAddrs;
    for(p = info; p != NULL; p = p->ai_next) {
        if (p->ai_family == AF_INET) { // IPv4
            struct sockaddr_in *ipv4 = (struct sockaddr_in *)p->ai_addr;
            /*
                This function converts the network address structure src in the
                af address family into a character string. The resulting string
                is copied to the buffer pointed to by dst, which must be a non-
                null pointer.  The caller specifies the number of bytes available
                in this buffer in the argument size.
                - https://man7.org/linux/man-pages/man3/inet_ntop.3.html
            */
            inet_ntop(p->ai_family, &(ipv4->sin_addr), ipstr, sizeof ipstr);
            ipAddrs.push_back(ipstr);
        }
    }

    freeaddrinfo(info); // free the linked list

    return ipAddrs;
}

std::string getPublicIPv4Address() {
    std::vector<std::string> ipAddrs = getLocalIPv4Addresses();
    if (ipAddrs.size() == 0) {
        return "";
    }

    // Return first IP address not equal to loopback address
    for (const std::string& ip : ipAddrs) {
        if (ip != "127.0.0.1") {
            return ip;
        }
    }
    return "";
}

std::string addTab(const std::string& str, int tabCount) {
    // Insert tabCount number of tabs at the beginning of each line
    std::string tab(tabCount, '\t');
    std::string result = tab + str;
    size_t pos = 2;
    while ((pos = result.find("\n", pos)) != std::string::npos) {
        result.replace(pos, 1, "\n" + tab);
        pos += tabCount + 1;
    }
    return result;
}