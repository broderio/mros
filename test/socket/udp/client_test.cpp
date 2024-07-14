#include "utils.hpp"
#include "socket/udp/client.hpp"    

int main() {
    UDPClient client;
    std::string ipAddr = getPublicIPv4Address();
    URI serverURI(ipAddr, 8080);

    std::string message;
    for (int i = 0; i < 10; i++) {
        message = "Message #" + std::to_string(i);
        client.sendTo(message, serverURI);
        sleep(1000);

        URI serverURI;
        int res = client.receiveFrom(message, 1024, serverURI);
        if (res < 0) {
            return 1;
        }
        if (message.size() == 0) {
            i--;
            continue;
        }

        std::cout << "Received: \"" << message << "\" from server at " << serverURI.toString() << std::endl;
    }
    return 0;
}