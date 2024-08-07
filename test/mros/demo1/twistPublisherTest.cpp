#include <iostream>
#include <string>

#include "utils.hpp"

#include "messages/geometry_msgs/twist.hpp"

#include "mros/common.hpp"
#include "mros/utils/console.hpp"
#include "mros/core/node.hpp"
#include "mros/core/publisher.hpp"

using namespace mros;

int main()
{
    Node &node = Node::getInstance();
    node.init("twist_publisher", URI(getLocalIP(), MEDIATOR_PORT_NUM));
    Console::setLevel(LogLevel::DEBUG);

    std::shared_ptr<Publisher> pub = node.advertise<geometry_msgs::TwistStamped>("twist_topic", 10);

    if (pub == nullptr)
    {
        std::cout << "Failed to advertise topic" << std::endl;
        return 1;
    }

    node.spin(false);

    int i = 0;
    while (node.ok())
    {
        geometry_msgs::TwistStamped msg;
        msg.header.frame_id = "robot";
        msg.twist.linear.x = 0.1 * (i % 6);
        msg.twist.angular.z = 0.1 * (i % 6);
        pub->publish(msg);

        sleep(100);
        ++i;
    }

    return 0;
}