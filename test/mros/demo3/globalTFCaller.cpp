#include <iostream>
#include <string>

#include "utils.hpp"

#include "messages/std_msgs/void.hpp"
#include "messages/geometry_msgs/transform.hpp"

#include "mros/utils/console.hpp"
#include "mros/core/node.hpp"
#include "mros/core/service.hpp"

using namespace mros;

int main()
{
    Node &node = Node::getInstance();
    node.init("service_node", URI(getLocalIP(), MEDIATOR_PORT_NUM));

    Console::setLevel(LogLevel::DEBUG);

    node.spin(false);

    while (node.ok())
    {
        geometry_msgs::TF response;
        Console::log(LogLevel::DEBUG, "Sending request ...");
        if (node.callService("global_tf", std_msgs::Void(), response))
        {
            Console::log(LogLevel::DEBUG, response.toString());
        }

        sleep(1000);
    }

    return 0;
}