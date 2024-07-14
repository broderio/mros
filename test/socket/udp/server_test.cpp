#include "utils.hpp"
#include "socket/udp/server.hpp"

int main() {
    UDPServer server(URI(getPublicIPv4Address(), 8080));

    int res = server.bind();
    if (res < 0) {
        return 1;
    }

    URI clientURI;
    std::string message;
    for (int i = 0; i < 10; i++) {
        res = server.receiveFrom(message, 1024, clientURI);
        if (res < 0) {
            return 1;
        }
        if (message.size() == 0) {
            i--;
            continue;
        }

        std::cout << "Received: \"" << message << "\" from " << clientURI << std::endl;

        server.sendTo(message, clientURI);
    }
    
    return 0;
}