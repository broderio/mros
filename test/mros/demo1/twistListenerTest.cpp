#include <iostream>
#include <string>

#include "utils.hpp"

#include "messages/geometry_msgs/twist.hpp"

#include "mros/common.hpp"
#include "mros/utils/console.hpp"
#include "mros/core/node.hpp"
#include "mros/core/subscriber.hpp"

using namespace mros;

int main()
{
    Node &node = Node::getInstance();
    node.init("twist_listener", URI(getLocalIP(), MEDIATOR_PORT_NUM));
    Console::setLevel(LogLevel::DEBUG);

    std::function<void(const geometry_msgs::TwistStamped &)> callback = [](const geometry_msgs::TwistStamped &msg)
    {
        std::string vx = std::to_string(msg.twist.linear.x.data);
        std::string vy = std::to_string(msg.twist.linear.y.data);
        std::string wz = std::to_string(msg.twist.angular.z.data);
        Console::log(LogLevel::INFO, "vx: " + vx + ", vy: " + vy + ", wz: " + wz);
    };

    std::shared_ptr<Subscriber> sub = node.subscribe("twist_topic", 10, callback);

    if (sub == nullptr)
    {
        std::cout << "Failed to subscribe to topic" << std::endl;
        return 1;
    }

    node.spin();

    return 0;
}