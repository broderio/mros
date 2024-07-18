#include "mros/node.hpp"
#include "mros/publisher.hpp"
#include "mros/console.hpp"
#include "utils.hpp"

#include "messages/sensor_msgs/jointState.hpp"

#include "kineval/json.hpp"
#include "kineval/joint.hpp"
#include "kineval/link.hpp"
#include "kineval/tree.hpp"

#include <iostream>
#include <string>

int main() {
    URI uri;
    uri.ip = "0.0.0.0";
    uri.port = MEDIATOR_PORT_NUM;

    mros::Node node("publisher_node", uri);

    std::shared_ptr<mros::Publisher> pub = node.advertise<sensor_msgs::JointState>("joint_group_1", 10);

    if (pub == nullptr) {
        std::cout << "Failed to advertise topic" << std::endl;
        return 1;
    }

    std::ifstream file("/Users/broderio/Repositories/simple_pubsub/robots/simpleBot.json");
    auto json = kineval::JsonParser::parse(file);
    kineval::KinematicTree kt = kineval::KinematicTree::fromJson(json->as_object());

    node.spin(true);

    const int hz = 10;
    const int periodNs = (1 / (float)hz) * 1000000000;

    int64_t start_time = getTimeNano();
    double state = 0.0;
    double dstate = M_PI / 180;
    while (node.ok())
    {
        if (getTimeNano() - start_time >= periodNs)
        {
            sensor_msgs::JointState msg;
            msg.header.stamp = std_msgs::Time::getTimeNow();
            auto it = kt.getJointNames().begin();
            std::string jointName = *it;
            msg.name.push_back(jointName);
            msg.position.push_back(state);
            msg.velocity.push_back(0.0);
            msg.effort.push_back(0.0);

            pub->publish(msg);

            state += dstate;
            if (state >= M_PI / 2 || state <= 0)
            {
                dstate *= -1;
            }
            state = std::max(0.0, std::min(M_PI / 2, state));

            mros::Console::log(mros::LogLevel::DEBUG, "Published [" + jointName + "] state: " + std::to_string(state));
            
            start_time = getTimeNano();
        }
    }

    return 0;
}