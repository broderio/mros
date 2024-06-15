#include "smem/squeue.hpp"
#include "utils.hpp"

#include <iostream>
#include <string>

int main() {
    SQueue squeue("/tmp", 3, 256);

    std::string msg;
    for (int i = 0; i < 10; i++) {
        msg = "Hello " + std::to_string(i);
        squeue.push(&msg[0], 7);
        std::cout << "Sent: " << msg << std::endl;
        sleep(1000);
    }
    return 0;
}