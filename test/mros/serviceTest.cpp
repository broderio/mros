#include <iostream>
#include <string>

#include "utils.hpp"

#include "mros/core/node.hpp"
#include "mros/core/service.hpp"

#include "messages/std_msgs/string.hpp"
#include "messages/std_msgs/integer.hpp"

void serviceCallback(const std_msgs::Int32 &req, std_msgs::String &res) {
    switch (req.data)
    {
    case 0:
        res.data = "zero";
        break;
    case 1:
        res.data = "one";
        break;
    case 2:
        res.data = "two";
        break;
    case 3:
        res.data = "three";
        break;
    case 4:
        res.data = "four";
        break;
    case 5:
        res.data = "five";
        break;
    case 6:
        res.data = "six";
        break;
    case 7:
        res.data = "seven";
        break;
    case 8:
        res.data = "eight";
        break;
    case 9:
        res.data = "nine";
        break;
    default:
        res.data = "unknown";
        break;
    }
}

int main() {
    URI uri;
    uri.ip = "0.0.0.0";
    uri.port = MEDIATOR_PORT_NUM;

    mros::Node node("service_node", uri);

    mros::Console::setLevel(mros::LogLevel::DEBUG);

    std::shared_ptr<mros::Service> srv = node.advertiseService<std_msgs::Int32, std_msgs::String>("test_service", &serviceCallback);

    if (srv == nullptr) {
        std::cout << "Failed to advertise service" << std::endl;
        return 1;
    }

    node.spin();

    return 0;
}