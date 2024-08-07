#include <iostream>
#include <string>

#include "utils.hpp"

#include "messages/std_msgs/string.hpp"

#include "mros/core/node.hpp"
#include "mros/core/subscriber.hpp"

using namespace mros;

int main()
{
    Node &node = Node::getInstance();
    node.init("subscriber_node", URI(getLocalIP(), MEDIATOR_PORT_NUM));

    std::function<void(const std_msgs::String &)> callback = [](const std_msgs::String &msg)
    {
        std::cout << "Received message: " << msg.data << std::endl;
    };

    std::shared_ptr<Subscriber> sub = node.subscribe("test_topic", 10, callback);

    if (sub == nullptr)
    {
        std::cout << "Failed to subscribe to topic" << std::endl;
        return 1;
    }

    node.spin();

    return 0;
}