#include <iostream>
#include <string>

#include "utils.hpp"
#include "socket/tcp/server.hpp"

int main() {
    TCPServer server(URI(getLocalIP(), 8080));
    server.bind();
    server.listen();
    TCPConnection connection;
    int res;
    do {
        res = server.accept(connection);
        if (res < 0) {
            return 1;
        }
    } while (res != 0);

    std::string message;
    for (int i = 0; i < 10; i++) {
        sleep(1000);
        message.clear();
        while (message.empty()) {
            res = connection.receive(message, 25);
            if (res < 0) {
                return 1;
            }
        }
        std::cout << "Received: \"" << message << "\" from " << connection.getClientURI() << std::endl;
        connection.send("Response #" + std::to_string(i));
    }

    return 0;
}