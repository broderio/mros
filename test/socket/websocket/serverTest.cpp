#include <iostream>
#include <string>

#include "utils.hpp"
#include "socket/websocket/server.hpp"

int main() {
    WSServer server(URI(getLocalIP(), 8080));
    std::cout << server.getURI() << std::endl;
    std::shared_ptr<WSConnection> connection;
    int res = -1;
    while (res != 0)
    {
        res = server.accept(connection);
        sleep(500);
    }

    std::string message;
    for (int i = 0; i < 3; i++) {
        sleep(1000);
        while (message.empty()) {
            res = connection->receive(message);
            if (res < 0) {
                return 1;
            }
        }
        std::cout << "Received: \"" << message << "\" from " << connection->getClientURI() << std::endl;
        connection->send("Response #" + std::to_string(i));
        message.clear();
    }

    return 0;
}