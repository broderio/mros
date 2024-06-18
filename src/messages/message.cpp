#include "messages/message.hpp"

std::ostream& operator<<(std::ostream& os, const IMessage& msg) {
    os << msg.toString();
    return os;
}