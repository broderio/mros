#include "mros/core/node.hpp"
#include "mros/core/publisher.hpp"
#include "utils.hpp"

#include "messages/std_msgs/void.hpp"
#include "messages/geometry_msgs/transform.hpp"

#include <iostream>
#include <string>

int main() {
    URI uri;
    uri.ip = "0.0.0.0";
    uri.port = MEDIATOR_PORT_NUM;

    mros::Node node("service_node", uri);

    mros::Console::setLevel(mros::LogLevel::DEBUG);

    node.spin(true);

    while (node.ok())
    {
        geometry_msgs::TF response;
        mros::Console::log(mros::LogLevel::DEBUG, "Sending request ...");
        if (node.callService("global_tf", std_msgs::Void(), response))
        {
            mros::Console::log(mros::LogLevel::DEBUG, response.toString());
        }

        sleep(1000);
    }

    return 0;
}