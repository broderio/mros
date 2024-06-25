#ifndef URI_MSG_HPP
#define URI_MSG_HPP

#include "utils.hpp"

#include "messages/std_msgs/string.hpp"
#include "messages/std_msgs/header.hpp"

namespace private_msgs {

class URI : public IMessage {
public:
    String hostname;
    int port;

    URI();

    URI(const String& hostname, const int& port);

    URI(const URI& other);

    URI& operator=(const URI& other);

    ::URI toURI() const;

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

class URIStamped : public IMessage {
public:
    Header header;
    URI uri;

    URIStamped();

    URIStamped(const Header& header, const URI& uri);

    URIStamped(const URIStamped& other);

    URIStamped& operator=(const URIStamped& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

class URIArray : IMessage {
public:
    Header header;
    std::vector<URI> uriArray;

    URIArray();

    URIArray(const Header& header, const std::vector<URI>& uriArray);

    URIArray(const URIArray& other);

    URIArray& operator=(const URIArray& other);

    uint16_t getMsgLen() const override;

    std::string toString() const override;

    std::string encode() const override;

    void decode(const std::string& msg) override;
};

}

#endif // URI_MSG_HPP