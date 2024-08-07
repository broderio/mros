#include <iostream>
#include <string>

#include "utils.hpp"

#include "messages/std_msgs/string.hpp"

#include "mros/core/node.hpp"
#include "mros/core/publisher.hpp"
#include "mros/core/subscriber.hpp"

using namespace mros;

int main()
{
    Node &node = node.getInstance();
    node.init("publisher_node", URI(getLocalIP(), MEDIATOR_PORT_NUM));

    std::shared_ptr<Publisher> pub = node.advertise<String>("test_topic", 10);
    if (pub == nullptr)
    {
        std::cout << "Failed to advertise topic" << std::endl;
        return 1;
    }

    std::shared_ptr<Subscriber> sub = node.subscribe<String>("test_topic", 10, [](const String &msg)
                                                              { std::cout << "Received message: " << msg.data << std::endl; });
    if (sub == nullptr)
    {
        std::cout << "Failed to subscribe to topic" << std::endl;
        return 1;
    }

    while (sub->getNumPublishers() == 0)
    {
        node.spinOnce();
    }

    for (int i = 0; i < 10; i++)
    {
        if (!node.ok())
        {
            break;
        }

        std_msgs::String msg;
        msg.data = "Message #" + std::to_string(i);
        pub->publish(msg);

        node.spinOnce();
        sleep(1000);
    }

    return 0;
}