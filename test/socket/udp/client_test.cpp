#include "utils.hpp"
#include "socket/udp/client.hpp"    

int main() {
    std::string ipAddr = getIPAddr();
    UDPClient client(ipAddr, 8080);

    int res = client.connect();
    if (res < 0) {
        return 1;
    }

    std::string message;
    for (int i = 0; i < 10; i++) {
        message = "Message #" + std::to_string(i);
        client.sendTo(message);
        sleep(1000);

        res = client.receiveFrom(message, 1024);
        if (res < 0) {
            return 1;
        }

        std::cout << "Received: \"" << message << "\" from server" << std::endl;
    }
    return 0;
}