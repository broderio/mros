#include "smem/squeue.hpp"

#include <iostream>
#include <string>

int main() {
    SQueue squeue("/tmp", 3, 256);
    char buffer[8];

    int i = 0;
    while (i < 10) {
        if (!squeue.empty()) {
            squeue.pop(buffer, 7);
            buffer[7] = '\0';
            std::cout << "Received: " << std::string(buffer) << std::endl;
            i++;
        }
    }

    return 0;
}