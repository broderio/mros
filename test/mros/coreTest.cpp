#include "mros/core/mediator.hpp"

#include <iostream>
#include <string>

int main() {
    mros::Mediator mediator;
    mros::Console::setLevel(mros::LogLevel::DEBUG);
    mediator.spin();
    return 0;
}