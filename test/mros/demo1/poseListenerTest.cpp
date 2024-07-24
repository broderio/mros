#include "mros/core/node.hpp"
#include "mros/core/subscriber.hpp"
#include "mros/utils/console.hpp"

#include "utils.hpp"

#include "messages/geometry_msgs/pose.hpp"

#include <iostream>
#include <string>

int main() {
    URI uri;
    uri.ip = "0.0.0.0";
    uri.port = MEDIATOR_PORT_NUM;

    mros::Node node("pose_listener", uri);

    std::function<void(const geometry_msgs::PoseStamped&)> callback = [](const geometry_msgs::PoseStamped &msg) {
        std::string x = std::to_string(msg.pose.position.x.data);
        std::string y = std::to_string(msg.pose.position.y.data);
        std::string z = std::to_string(msg.pose.position.z.data);
        std::string qx = std::to_string(msg.pose.orientation.x.data);
        std::string qy = std::to_string(msg.pose.orientation.y.data);
        std::string qz = std::to_string(msg.pose.orientation.z.data);
        std::string qw = std::to_string(msg.pose.orientation.w.data);
        mros::Console::log(mros::LogLevel::DEBUG, "xyz: [" + x + ", " + y + ", " + z + "], q: [" + qx + ", " + qy + ", " + qz + ", " + qw + "]");
    };

     std::shared_ptr<mros::Subscriber> sub = node.subscribe("pose_topic", 10, callback);

    if (sub == nullptr) {
        std::cout << "Failed to subscribe to topic" << std::endl;
        return 1;
    }

    node.spin();

    return 0;
}