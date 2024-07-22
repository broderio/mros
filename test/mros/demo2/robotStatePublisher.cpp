#include <iostream>
#include <string>
#include <mutex>

#include "socket/tcp/client.hpp"

#include "mros/node.hpp"
#include "mros/subscriber.hpp"
#include "mros/console.hpp"
#include "utils.hpp"

#include "messages/sensor_msgs/jointState.hpp"
#include "messages/geometry_msgs/transform.hpp"

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "linalg/quaternion.hpp"

#include "kineval/tree.hpp"

sensor_msgs::JointState interpolate(const sensor_msgs::JointState &js1, const sensor_msgs::JointState &js2, double targetTime)
{
    sensor_msgs::JointState result;
    double t1 = js1.header.stamp.sec.data + js1.header.stamp.nsec.data / 1000000000.0;
    double t2 = js2.header.stamp.sec.data + js2.header.stamp.nsec.data / 1000000000.0;
    double ratio = (targetTime - t1) / (t2 - t1);

    int32_t seconds = std::floor(targetTime);
    result.header.stamp.sec.data = seconds;
    result.header.stamp.nsec.data = (targetTime - seconds) * 1000000000;

    for (int i = 0; i < js1.name.size(); i++)
    {
        result.name.push_back(js1.name[i]);
        result.position.push_back(js1.position[i].data + ratio * (js2.position[i].data - js1.position[i].data));
        result.velocity.push_back(js1.velocity[i].data + ratio * (js2.velocity[i].data - js1.velocity[i].data));
        result.effort.push_back(js1.effort[i].data + ratio * (js2.effort[i].data - js1.effort[i].data));
    }

    if (js2.name.size() > js1.name.size())
    {
        for (int i = js1.name.size(); i < js2.name.size(); i++)
        {
            result.name.push_back(js2.name[i]);
            result.position.push_back(js2.position[i].data);
            result.velocity.push_back(js2.velocity[i].data);
            result.effort.push_back(js2.effort[i].data);
        }
    }

    return result;
}

void updateJointStates(kineval::KinematicTree &kt, const sensor_msgs::JointState &js)
{
    for (int i = 0; i < js.name.size(); i++)
    {
        std::string name = js.name[i].data;
        mros::Console::log(mros::LogLevel::DEBUG, "Updating joint [" + name + "] to position: " + std::to_string(js.position[i].data));
        double position = js.position[i].data;
        kt.setJointState(name, position);
    }
}

class RobotStatePublisher
{
public:
    RobotStatePublisher(const URI &uri, const std::string &jrdfPath, int hz);

    void run();

private:
    void jointStateCallback(const sensor_msgs::JointState &msg);

    int hz;
    mros::Node node;
    std::shared_ptr<mros::Subscriber> jointStateSub;
    std::shared_ptr<mros::Publisher> tfPub;

    sensor_msgs::JointState currJointState;
    sensor_msgs::JointState prevJointState;

    kineval::KinematicTree kt;
};

void RobotStatePublisher::jointStateCallback(const sensor_msgs::JointState &msg)
{
    prevJointState = currJointState;
    currJointState = msg;
}

RobotStatePublisher::RobotStatePublisher(const URI &uri, const std::string &jrdfPath, int hz)
    : node("robot_state_publisher", uri), hz(hz)
{
    tfPub = node.advertise<geometry_msgs::TF>("tf", 10);
    jointStateSub = node.subscribe<sensor_msgs::JointState>("joint_states", 10, std::bind(&RobotStatePublisher::jointStateCallback, this, std::placeholders::_1));

    std::ifstream file(jrdfPath);
    auto json = kineval::JsonParser::parse(file);
    kt = kineval::KinematicTree::fromJson(json->as_object());
}

void RobotStatePublisher::run()
{
    TCPClient client(URI("0.0.0.0", 9000), false);
    client.connect();

    const int periodNs = (1 / (float)hz) * 1000000000;

    int64_t start_time = getTimeNano();
    while (node.ok())
    {
        if (getTimeNano() - start_time >= periodNs)
        {
            double targetTime = getTimeNano() / 1000000000.0;
            sensor_msgs::JointState interpolatedState = interpolate(prevJointState, currJointState, targetTime);

            updateJointStates(kt, interpolatedState);

            tfPub->publish(kt.TF());
            start_time = getTimeNano();
        }
        node.spinOnce();
    }
}

int main()
{
    URI uri;
    uri.ip = "0.0.0.0";
    uri.port = MEDIATOR_PORT_NUM;

    std::string jrdfPath = "/Users/broderio/Repositories/simple_pubsub/robots/simpleBot.json";

    RobotStatePublisher RSP(uri, jrdfPath, 50);
    RSP.run();

    return 0;
}