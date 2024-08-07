#include <iostream>
#include <string>
#include <mutex>

#include "utils.hpp"
#include "socket/tcp/client.hpp"

#include "messages/std_msgs/void.hpp"
#include "messages/sensor_msgs/jointState.hpp"
#include "messages/geometry_msgs/transform.hpp"

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "linalg/quaternion.hpp"

#include "mros/utils/console.hpp"
#include "mros/utils/argParser.hpp"
#include "mros/core/node.hpp"
#include "mros/core/subscriber.hpp"
#include "mros/core/publisher.hpp"
#include "mros/core/service.hpp"

#include "kineval/tree.hpp"

using namespace mros;

sensor_msgs::JointState interpolate(const sensor_msgs::JointState &js1, const sensor_msgs::JointState &js2, double targetTime, float timeThreshold)
{
    sensor_msgs::JointState result;
    double t1 = js1.header.stamp.sec.data + js1.header.stamp.nsec.data / 1000000000.0;
    double t2 = js2.header.stamp.sec.data + js2.header.stamp.nsec.data / 1000000000.0;

    // If the target time is too far ahead of the current time, just return the latest joint state
    if (targetTime - t2 > timeThreshold)
    {
        return js2;
    }

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

class RobotStatePublisher
{
public:
    RobotStatePublisher(const URI &uri, const std::string &jrdfPath, int hz, float timeThreshold);

    void run();

private:
    void jointStateCallback(const sensor_msgs::JointState &msg);
    void globalTFCallback(const std_msgs::Void &req, geometry_msgs::TF &res);

    int hz;
    float timeThreshold;
    std::shared_ptr<Subscriber> jointStateSub;
    std::shared_ptr<Publisher> tfPub;
    std::shared_ptr<Service> globalTFService;

    sensor_msgs::JointState currJointState;
    sensor_msgs::JointState prevJointState;

    std::mutex ktLock;
    kineval::KinematicTree kt;
};

void RobotStatePublisher::jointStateCallback(const sensor_msgs::JointState &msg)
{
    prevJointState = currJointState;
    currJointState = msg;
}

void RobotStatePublisher::globalTFCallback(const std_msgs::Void &req, geometry_msgs::TF &res)
{
    this->ktLock.lock();
    res = this->kt.globalTF();
    this->ktLock.unlock();
}

RobotStatePublisher::RobotStatePublisher(const URI &uri, const std::string &jrdfPath, int hz, float timeThreshold)
    : hz(hz), timeThreshold(timeThreshold)
{
    Node &node = Node::getInstance();
    node.init("robot_state_publisher", uri);

    tfPub = node.advertise<geometry_msgs::TF>("tf", 10);
    if (tfPub == nullptr)
    {
        Console::log(LogLevel::ERROR, "Failed to advertise tf topic");
        return;
    }

    using namespace std::placeholders;
    
    jointStateSub = node.subscribe<sensor_msgs::JointState>("joint_states", 10, std::bind(&RobotStatePublisher::jointStateCallback, this, _1));
    if (jointStateSub == nullptr)
    {
        Console::log(LogLevel::ERROR, "Failed to subscribe to joint_states topic");
        return;
    }

    globalTFService = node.advertiseService<std_msgs::Void, geometry_msgs::TF>("global_tf", std::bind(&RobotStatePublisher::globalTFCallback, this, _1, _2));
    if (globalTFService == nullptr)
    {
        Console::log(LogLevel::ERROR, "Failed to advertise global_tf service");
        return;
    }

    std::ifstream file(jrdfPath);
    auto json = kineval::JsonParser::parse(file);
    kt = kineval::KinematicTree::fromJson(json->as_object());
}

void RobotStatePublisher::run()
{
    Node &node = Node::getInstance();
    node.spin(false);

    const int periodNs = (1 / (float)hz) * 1000000000;

    int64_t start_time = getTimeNano();
    while (node.ok())
    {
        if (getTimeNano() - start_time >= periodNs)
        {
            if (jointStateSub->getNumPublishers() == 0)
            {
                Console::log(LogLevel::DEBUG, "No publishers for joint_states. Stopping interpolation.");
                tfPub->publish(kt.TF());
                start_time = getTimeNano();
                continue;
            }

            double targetTime = getTimeNano() / 1000000000.0;

            sensor_msgs::JointState interpolatedState = interpolate(prevJointState, currJointState, targetTime, timeThreshold);

            ktLock.lock();
            kt.setJointStates(interpolatedState);
            tfPub->publish(kt.TF());
            ktLock.unlock();

            start_time = getTimeNano();
        }
    }
}

int main(int argc, char **argv)
{
    ArgParser::init("robot_state_publisher", "Publishes interpolated transforms based on joint states");

    ArgParser::addArg({"jrdf", "Path to JRDF file", '1'});

    ArgParser::addOpt({"ip", "i", "Core IP address", "0.0.0.0", "", '1'});
    ArgParser::addOpt({"freq", "f", "Publish rate in Hz", "25", "", '1'});
    ArgParser::addOpt({"delta-time", "d", "Time threshold to stop interpolation", "0.1", "", '1'});

    ArgParser::parse(argc, argv);

    std::string jrdfPath = ArgParser::getArg("jrdf")[0];

    URI uri;
    uri.ip = ArgParser::getOpt("ip")[0];
    uri.port = MEDIATOR_PORT_NUM;

    int freq = std::stoi(ArgParser::getOpt("freq")[0]);

    float timeThreshold = std::stof(ArgParser::getOpt("delta-time")[0]);

    RobotStatePublisher RSP(uri, jrdfPath, freq, timeThreshold);
    Console::setLevel(LogLevel::DEBUG);
    RSP.run();

    return 0;
}