#include "smem/smutex.hpp"
#include "smem/smem.hpp"
#include "utils.hpp"

#include <iostream>
#include <string>

int main() {
    SMutex smutex("/tmp", 1);
    SMem smem("/tmp", 2, 4);
    smem.attach();

    std::string msg;
    int i = 0;
    while (true) {
        smutex.lock();
        int *ptr = (int*)smem.get();
        std::cout << "mutex locked! value: " << *ptr << std::endl;
        *ptr = i++;
        smutex.unlock();
        std::cout << "mutex unlocked!" << std::endl;
        sleep(1000);
    }
    return 0;
}