#include "mros/core/node.hpp"
#include "mros/core/subscriber.hpp"
#include "utils.hpp"

#include "messages/geometry_msgs/twist.hpp"

#include <iostream>
#include <string>

int main() {
    URI uri;
    uri.ip = "0.0.0.0";
    uri.port = MEDIATOR_PORT_NUM;

    mros::Node node("twist_listener", uri);

    std::function<void(const geometry_msgs::TwistStamped&)> callback = [](const geometry_msgs::TwistStamped &msg) {
        std::string vx = std::to_string(msg.twist.linear.x.data);
        std::string vy = std::to_string(msg.twist.linear.y.data);
        std::string wz = std::to_string(msg.twist.angular.z.data);
        mros::Console::log(mros::LogLevel::DEBUG, "vx: " + vx + ", vy: " + vy + ", wz: " + wz);
    };

     std::shared_ptr<mros::Subscriber> sub = node.subscribe("twist_topic", 10, callback);

    if (sub == nullptr) {
        std::cout << "Failed to subscribe to topic" << std::endl;
        return 1;
    }

    node.spin();

    return 0;
}