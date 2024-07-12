#include "mros/private_msgs/uri.hpp"

namespace private_msgs {

URI::URI() : hostname(), port(0) {}

URI::URI(const String& hostname, const Int32& port)
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
    return ::URI(hostname.data, port.data);
}

uint16_t URI::getMsgLen() const {
    return hostname.getMsgLen() + port.getMsgLen();
}

std::string URI::toString() const {
    std::stringstream ss;
    ss << "hostname:\n" << addTab(hostname.toString(), 1) << "\n";
    ss << "port:\n" << addTab(std::to_string(port.data), 1) << "\n";
    return ss.str();
}

std::string URI::encode() const {
    std::string msg;
    msg.append(hostname.encode());
    msg.append(port.encode());
    return msg;
}

bool URI::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a URI." << std::endl;
        return false;
    }

    int len = 0;
    if (!hostname.decode(msg)) {
        std::cerr << "Error: could not decode hostname." << std::endl;
        return false;
    }
    len += hostname.getMsgLen();

    if (!port.decode(msg.substr(len))) {
        std::cerr << "Error: could not decode port." << std::endl;
        return false;
    }

    return true;
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
    ss << "header:\n" << addTab(header.toString(), 1) << "\n";
    ss << "uri:\n" << addTab(uri.toString(), 1) << "\n";
    return ss.str();
}

std::string URIStamped::encode() const {
    std::string msg;
    msg.append(header.encode());
    msg.append(uri.encode());
    return msg;
}

bool URIStamped::decode(const std::string& msg) {
    if (msg.size() < getMsgLen()) {
        std::cerr << "Error: message is too short to be a URIStamped." << std::endl;
        return false;
    }

    int len = 0;
    if (!header.decode(msg)) {
        std::cerr << "Error: could not decode header." << std::endl;
        return false;
    }
    len += header.getMsgLen();

    if (!uri.decode(msg.substr(len))) {
        std::cerr << "Error: could not decode uri." << std::endl;
        return false;
    }

    return true;
}

}