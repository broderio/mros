#include <iostream>
#include <string>

#include "utils.hpp"

#include "messages/geometry_msgs/pose.hpp"

#include "mros/common.hpp"
#include "mros/utils/console.hpp"
#include "mros/core/node.hpp"
#include "mros/core/subscriber.hpp"

using namespace mros;

int main()
{
    Node &node = Node::getInstance();
    node.init("pose_listener", URI(getLocalIP(), MEDIATOR_PORT_NUM));
    Console::setLevel(LogLevel::DEBUG);

    std::function<void(const geometry_msgs::PoseStamped &)> callback = [](const geometry_msgs::PoseStamped &msg)
    {
        std::string x = std::to_string(msg.pose.position.x.data);
        std::string y = std::to_string(msg.pose.position.y.data);
        std::string z = std::to_string(msg.pose.position.z.data);
        std::string qx = std::to_string(msg.pose.orientation.x.data);
        std::string qy = std::to_string(msg.pose.orientation.y.data);
        std::string qz = std::to_string(msg.pose.orientation.z.data);
        std::string qw = std::to_string(msg.pose.orientation.w.data);
        Console::log(LogLevel::DEBUG, "xyz: [" + x + ", " + y + ", " + z + "], q: [" + qx + ", " + qy + ", " + qz + ", " + qw + "]");
    };

    std::shared_ptr<Subscriber> sub = node.subscribe("pose_topic", 10, callback);

    if (sub == nullptr)
    {
        std::cout << "Failed to subscribe to topic" << std::endl;
        return 1;
    }

    node.spin();

    return 0;
}