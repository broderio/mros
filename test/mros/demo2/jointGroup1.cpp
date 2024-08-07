#include <iostream>
#include <string>

#include "utils.hpp"

#include "messages/sensor_msgs/jointState.hpp"

#include "mros/utils/console.hpp"
#include "mros/core/node.hpp"
#include "mros/core/publisher.hpp"

#include "kineval/json.hpp"
#include "kineval/joint.hpp"
#include "kineval/link.hpp"
#include "kineval/tree.hpp"

using namespace mros;

int main() {
    Node &node = Node::getInstance();
    node.init("publisher_node", URI(getLocalIP(), MEDIATOR_PORT_NUM));

    Console::setLevel(LogLevel::DEBUG);

    std::shared_ptr<Publisher> pub = node.advertise<sensor_msgs::JointState>("joint_group_1", 10);

    if (pub == nullptr) {
        std::cout << "Failed to advertise topic" << std::endl;
        return 1;
    }

    std::ifstream file("/Users/broderio/Repositories/simple_pubsub/robots/simpleBot.json");
    auto json = kineval::JsonParser::parse(file);
    kineval::KinematicTree kt = kineval::KinematicTree::fromJson(json->as_object());

    node.spin(false);

    const int hz = 10;
    const int periodNs = (1 / (float)hz) * 1000000000;

    int64_t start_time = getTimeNano();
    double state = 0.0;
    double dstate = M_PI / 180;
    std::string jointName = kt.getJointNames()[1];

    double lowerBound = 0.0, upperBound = M_2_PI;
    bool hasLimits = kt.getJoint(jointName).getLimits(lowerBound, upperBound);
    while (node.ok())
    {
        if (getTimeNano() - start_time >= periodNs)
        {
            sensor_msgs::JointState msg;
            msg.header.stamp = std_msgs::Time::getTimeNow();
            msg.name.push_back(jointName);
            msg.position.push_back(state);
            msg.velocity.push_back(0.0);
            msg.effort.push_back(0.0);

            pub->publish(msg);

            state += dstate;
            if (hasLimits && (state >= upperBound || state <= lowerBound))
            {
                dstate *= -1;
            }
            state = std::max(lowerBound, std::min(upperBound, state));

            Console::log(LogLevel::DEBUG, "Published [" + jointName + "] state: " + std::to_string(state));
            
            start_time = getTimeNano();
        }
    }

    return 0;
}