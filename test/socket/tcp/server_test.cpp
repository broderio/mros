#include <iostream>
#include <string>

#include "utils.hpp"
#include "socket/tcp/server.hpp"

int main() {
    TCPServer server(URI(getIPAddr(), 8080), 0);
    TCPConnection connection;
    int res = server.accept(connection);
    if (res < 0) {
        return 1;
    }

    std::string message;
    for (int i = 0; i < 10; i++) {
        sleep(1000);
        res = connection.receive(message, 25);
        if (res < 0) {
            return 1;
        }
        std::cout << "Received: \"" << message << "\" from " << connection.getClientURI() << std::endl;
        connection.send("Response #" + std::to_string(i));
    }

    return 0;
}