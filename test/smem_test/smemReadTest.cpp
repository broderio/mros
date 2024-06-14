#include "smem/sbuffer.hpp"

#include <iostream>
#include <string>

int main() {
    SBuffer sbuffer(1, 1024);

    int i = 0;
    while (i < 10) {
        std::string msg = sbuffer.read(10);
        if (msg.empty()) {
            continue;
        }
        std::cout << "Received: " << msg << std::endl;
        i++;
    }

    return 0;
}