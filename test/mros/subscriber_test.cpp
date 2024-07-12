#include "mros/node.hpp"
#include "mros/subscriber.hpp"
#include "utils.hpp"

#include "messages/std_msgs/string.hpp"

#include <iostream>
#include <string>

int main() {
    URI uri;
    uri.ip = "35.3.187.113";
    uri.port = MEDIATOR_PORT_NUM;

    mros::Node node("subscriber_node", uri);

    std::function<void(const std_msgs::String &)> callback = [](const std_msgs::String &msg) {
        std::cout << "Received message: " << msg.data << std::endl;
    };


    std::shared_ptr<mros::Subscriber> sub = node.subscribe("test_topic", 10, callback);

    if (sub == nullptr) {
        std::cout << "Failed to subscribe to topic" << std::endl;
        return 1;
    }

    node.spin();

    return 0;
}