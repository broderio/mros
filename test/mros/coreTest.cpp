#include "mros/mediator.hpp"

#include <iostream>
#include <string>

int main() {
    mros::Mediator mediator;
    mediator.spin();
    return 0;
}