#include "mros/core/node.hpp"
#include "mros/core/publisher.hpp"
#include "utils.hpp"

#include "messages/std_msgs/string.hpp"
#include "messages/std_msgs/integer.hpp"

#include <iostream>
#include <string>

int main() {
    URI uri;
    uri.ip = "0.0.0.0";
    uri.port = MEDIATOR_PORT_NUM;

    mros::Node node("service_node", uri);

    mros::Console::setLevel(mros::LogLevel::DEBUG);

    node.spin(true);

    for (int i = 0; i < 10; i++) {
        if (!node.ok()) {
            break;
        }

        std_msgs::Int32 request = std_msgs::Int32(i);
        std_msgs::String response;
        if (node.callService("test_service", request, response))
        {
            mros::Console::log(mros::LogLevel::DEBUG, response.data);
        }

        sleep(1000);
    }

    return 0;
}