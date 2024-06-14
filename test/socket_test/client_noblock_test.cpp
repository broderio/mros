#include <iostream>
#include <string>

#include "utils.hpp"
#include "socket/client.hpp"

int main() {
    std::string ipAddr = getIPAddr();
    TCPClient client(ipAddr, 8080);
    int res;
    do {
        res = client.connect();
        if (res < 0) {
            return 1;
        }
    } while (res != 0);

    std::string message;
    for (int i = 0; i < 10; i++) {
        client.send("Message #" + std::to_string(i));
        sleep(1000);
        message.clear();
        while (message.empty()) {
            res = client.receive(message, 25);
            if (res < 0) {
                return 1;
            }
        }
        std::cout << message << std::endl;
    }
    return 0;
}