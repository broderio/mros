#include <iostream>
#include <string>

#include "utils.hpp"
#include "socket/websocket/client.hpp"

int main() {
    std::string ipAddr = getLocalIP();
    WSClient client(URI(ipAddr, 8080));

    while (!client.isConnected())
    {
        client.connect();
        sleep(100);
    }

    int res;
    std::string message;
    for (int i = 0; i < 3; i++) {
        client.send("Message #" + std::to_string(i));
        sleep(1000);
        while (message.empty()) {
            res = client.receive(message);
            if (res < 0) {
                return 1;
            }
        }
        std::cout << "Received: \"" << message << "\" from " << client.getServerURI() << std::endl;
        message.clear();
    }
    return 0;
}