#include "utils.hpp"
#include "socket/udp/server.hpp"

int main() {
    UDPServer server(8081);

    int res = server.bind();
    if (res < 0) {
        return 1;
    }

    std::string ipAddr = getIPAddr();
    URI serverURI(ipAddr, 8080);

    std::string message;
    for (int i = 0; i < 10; i++) {
        message = "Message #" + std::to_string(i);
        server.sendTo(message, serverURI);
        sleep(1000);

        URI sender;
        int res = server.receiveFrom(message, 1024, sender);
        if (res < 0) {
            return 1;
        }
        if (message.size() == 0) {
            i--;
            continue;
        }

        std::cout << "Received: \"" << message << "\" from server at " << sender.toString() << std::endl;
    }
    return 0;
}