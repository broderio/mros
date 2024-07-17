#include "mros/node.hpp"
#include "mros/publisher.hpp"
#include "utils.hpp"

#include "messages/geometry_msgs/twist.hpp"

#include <iostream>
#include <string>

int main()
{
    URI uri;
    uri.ip = "0.0.0.0";
    uri.port = MEDIATOR_PORT_NUM;

    mros::Node node("twist_publisher", uri);

    std::shared_ptr<mros::Publisher> pub = node.advertise<geometry_msgs::TwistStamped>("twist_topic", 10);

    if (pub == nullptr)
    {
        std::cout << "Failed to advertise topic" << std::endl;
        return 1;
    }

    node.spin(true);

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