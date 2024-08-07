#include <iostream>
#include <string>

#include "utils.hpp"

#include "messages/std_msgs/string.hpp"
#include "messages/std_msgs/integer.hpp"

#include "mros/core/node.hpp"
#include "mros/core/publisher.hpp"

using namespace mros;

int main()
{
    Node &node = Node::getInstance();
    node.init("service_node", URI(getLocalIP(), MEDIATOR_PORT_NUM));

    Console::setLevel(LogLevel::DEBUG);

    node.spin(false);

    for (int i = -1; i < 11; i++)
    {
        if (!node.ok())
        {
            break;
        }

        std_msgs::Int32 request(i);
        std_msgs::String response;
        Console::log(LogLevel::DEBUG, "Sending request: " + std::to_string(request.data));
        if (node.callService("test_service", request, response))
        {
            Console::log(LogLevel::DEBUG, "Received response: " + response.data);
        }

        sleep(1000);
    }

    return 0;
}