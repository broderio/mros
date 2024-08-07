#include <iostream>
#include <string>

#include "utils.hpp"

#include "messages/std_msgs/string.hpp"

#include "mros/core/node.hpp"
#include "mros/core/publisher.hpp"

using namespace mros;

int main()
{
    Node &node = Node::getInstance();
    node.init("publisher_node", URI(getLocalIP(), MEDIATOR_PORT_NUM));

    std::shared_ptr<mros::Publisher> pub = node.advertise<String>("test_topic", 10);

    if (pub == nullptr)
    {
        std::cout << "Failed to advertise topic" << std::endl;
        return 1;
    }

    node.spin(false); // non-blocking

    for (int i = 0; i < 10; i++)
    {
        if (!node.ok())
        {
            break;
        }

        std_msgs::String msg;
        msg.data = "Message #" + std::to_string(i);
        pub->publish(msg);

        sleep(1000);
    }

    return 0;
}