#include "mros/node.hpp"
#include "mros/publisher.hpp"
#include "utils.hpp"

#include "messages/std_msgs/string.hpp"

#include <iostream>
#include <string>

int main() {
    URI uri;
    uri.ip = "0.0.0.0";
    uri.port = MEDIATOR_PORT_NUM;

    mros::Node node("publisher_node", uri);

    std::shared_ptr<mros::Publisher> pub = node.advertise<String>("test_topic", 10);

    if (pub == nullptr) {
        std::cout << "Failed to advertise topic" << std::endl;
        return 1;
    }

    while (pub->getNumSubscribers() == 0) {
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