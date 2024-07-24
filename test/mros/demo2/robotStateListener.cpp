#include <iostream>
#include <string>
#include <mutex>

#include "mros/core/node.hpp"
#include "mros/core/subscriber.hpp"
#include "mros/utils/console.hpp"
#include "utils.hpp"

#include "messages/sensor_msgs/jointState.hpp"
#include "messages/geometry_msgs/transform.hpp"

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "linalg/quaternion.hpp"

#include "kineval/tree.hpp"

int main() {
    URI uri;
    uri.ip = "0.0.0.0";
    uri.port = MEDIATOR_PORT_NUM;

    mros::Node node("robot_state", uri);
    mros::Console::setLevel(mros::LogLevel::DEBUG);

    std::function<void(const geometry_msgs::TF &)> callback = [](const geometry_msgs::TF &msg) {
        mros::Console::log(mros::LogLevel::DEBUG, msg.toString());
    };

    std::shared_ptr<mros::Subscriber> sub = node.subscribe("tf", 10, callback);

    if (sub == nullptr) {
        std::cout << "Failed to subscribe to topic" << std::endl;
        return 1;
    }

    node.spin();

    return 0;
}