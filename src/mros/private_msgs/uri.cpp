#include "mros/private_msgs/uri.hpp"

namespace private_msgs {

URI::URI() : hostname(), port(0) {}

URI::URI(const String& hostname, const int& port)
: hostname(hostname), port(port) {}

URI::URI(const URI& other) {
    hostname = other.hostname;
    port = other.port;
}

URI& URI::operator=(const URI& other) {
    if (this == &other) {
        return *this;
    }
    hostname = other.hostname;
    port = other.port;
    return *this;
}

::URI URI::toURI() const {
    return ::URI(hostname.data, port);
}

uint16_t URI::getMsgLen() const {
    return hostname.getMsgLen() + sizeof(port);
}

std::string URI::toString() const {
    std::stringstream ss;
    ss << hostname.data << ":" << port << "\n";
    return ss.str();
}

std::string URI::encode() const {
    std::string msg;
    msg.append(hostname.encode());
    msg.append((char*)&port, sizeof(port));
    return msg;
}

void URI::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a URI." << std::endl;
        return;
    }

    int len = 0;
    hostname.decode(msg); len += hostname.getMsgLen();
    std::memcpy(&port, msg.data() + len, sizeof(port));
}


URIStamped::URIStamped(): header(), uri() {}

URIStamped::URIStamped(const Header& header, const URI& uri)
: header(header), uri(uri) {}

URIStamped::URIStamped(const URIStamped& other) {
    header = other.header;
    uri = other.uri;
}

URIStamped& URIStamped::operator=(const URIStamped& other) {
    if (this == &other) {
        return *this;
    }

    header = other.header;
    uri = other.uri;
    return *this;
}

uint16_t URIStamped::getMsgLen() const {
    return header.getMsgLen() + uri.getMsgLen();
}

std::string URIStamped::toString() const {
    std::stringstream ss;
    ss << '\t' << header.toString() << "\n";
    ss << '\t' << uri.toString() << "\n";
    return ss.str();
}

std::string URIStamped::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(uri.encode());
    return msg;
}

void URIStamped::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a URIStamped." << std::endl;
        return;
    }

    int len = 0;
    header.decode(msg); len += header.getMsgLen();
    uri.decode(msg.substr(len));
}

}