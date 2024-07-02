#include "messages/message.hpp"

std::ostream& operator<<(std::ostream& os, const IMessage& msg) {
    os << msg.toString();
    return os;
}

char Parser::checksum(char* data, int len) {
    char sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return 255 - ( ( sum ) % 256 );
}   