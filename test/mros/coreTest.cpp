#include <iostream>
#include <string>

#include "mros/utils/console.hpp"
#include "mros/core/mediator.hpp"

using namespace mros;

int main()
{
    Mediator &mediator = Mediator::getInstance();
    mediator.init();
    Console::setLevel(LogLevel::DEBUG);
    mediator.spin();
    return 0;
}