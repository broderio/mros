#include "mros/node.hpp"
#include "mros/subscriber.hpp"
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
        std::cout << msg << std::endl;
    };

     std::shared_ptr<mros::Subscriber> sub = node.subscribe("pose_topic", 10, callback);

    if (sub == nullptr) {
        std::cout << "Failed to subscribe to topic" << std::endl;
        return 1;
    }

    node.spin();

    return 0;
}