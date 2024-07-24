#include "mros/core/node.hpp"
#include "mros/core/publisher.hpp"
#include "utils.hpp"

#include "messages/std_msgs/string.hpp"
#include "messages/std_msgs/integer.hpp"

#include <iostream>
#include <string>

void serviceCallback(const std_msgs::Int32 &req, std_msgs::String &res) {
    res = "Response to request: " + std::to_string(req.data);
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