#include "smem/sbuffer.hpp"
#include "utils.hpp"

#include <iostream>
#include <string>

int main() {
    SBuffer sbuffer(1, 1024);

    int i = 0;
    while (i < 10) {
        std::string msg = "Message #" + std::to_string(i);
        sbuffer.write(msg);
        std::cout << "Sent: " << msg << std::endl;
        sleep(1);
    }

    return 0;
}