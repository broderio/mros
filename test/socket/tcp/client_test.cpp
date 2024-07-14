#include <iostream>
#include <string>

#include "utils.hpp"
#include "socket/tcp/client.hpp"

int main() {
    std::string ipAddr = getPublicIPv4Address();
    TCPClient client(URI(ipAddr, 8080), 0);
    int res = client.connect();
    if (res < 0) {
        return 1;
    }

    std::string message;
    for (int i = 0; i < 10; i++) {
        client.send("Message #" + std::to_string(i));
        sleep(1000);
        res = client.receive(message, 25);
        if (res < 0) {
            return 1;
        }
        std::cout << "Received: \"" << message << "\" from " << client.getServerURI() << std::endl;
    }
    return 0;
}