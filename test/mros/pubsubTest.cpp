#include "mros/core/node.hpp"
#include "mros/core/publisher.hpp"
#include "mros/core/subscriber.hpp"
#include "utils.hpp"

#include "messages/std_msgs/string.hpp"

#include <iostream>
#include <string>

int main() {
    URI uri;
    uri.ip = "35.3.187.113";
    uri.port = MEDIATOR_PORT_NUM;

    mros::Node node("publisher_node", uri);

    std::shared_ptr<mros::Publisher> pub = node.advertise<String>("test_topic", 10);
    if (pub == nullptr) {
        std::cout << "Failed to advertise topic" << std::endl;
        return 1;
    }

    std::shared_ptr<mros::Subscriber> sub = node.subscribe<String>("test_topic", 10, [](const String &msg) {
        std::cout << "Received message: " << msg.data << std::endl;
    });
    if (sub == nullptr) {
        std::cout << "Failed to subscribe to topic" << std::endl;
        return 1;
    }

    while (sub->getNumPublishers() == 0) {
        node.spinOnce();
        sleep(10);
    }

    for (int i = 0; i < 10; i++) {
        if (!node.ok()) {
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