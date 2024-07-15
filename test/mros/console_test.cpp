#include "mros/console.hpp"

using namespace mros;

int main() {
    Console::init("test");
    Console::log(DEBUG, "This is a debug message");
    Console::log(INFO, "This is an info message");
    Console::log(WARN, "This is a warning message");
    Console::log(ERROR, "This is an error message");
    Console::log(FATAL, "This is a fatal message");

    sleep(2000);

    Console::log(DEBUG, "This is a debug message");
    Console::log(INFO, "This is an info message");
    Console::log(WARN, "This is a warning message");
    Console::log(ERROR, "This is an error message");
    Console::log(FATAL, "This is a fatal message");

    return 0;
}