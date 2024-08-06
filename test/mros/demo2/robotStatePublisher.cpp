#include <iostream>
#include <string>
#include <mutex>

#include "socket/tcp/client.hpp"

#include "mros/core/node.hpp"
#include "mros/core/subscriber.hpp"
#include "mros/utils/console.hpp"
#include "mros/utils/argParser.hpp"
#include "utils.hpp"

#include "messages/std_msgs/void.hpp"
#include "messages/sensor_msgs/jointState.hpp"
#include "messages/geometry_msgs/transform.hpp"

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "linalg/quaternion.hpp"

#include "kineval/tree.hpp"

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
    mros::Node node;
    std::shared_ptr<mros::Subscriber> jointStateSub;
    std::shared_ptr<mros::Publisher> tfPub;
    std::shared_ptr<mros::Service> globalTFService;

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
    : node("robot_state_publisher", uri), hz(hz), timeThreshold(timeThreshold)
{
    tfPub = node.advertise<geometry_msgs::TF>("tf", 10);
    jointStateSub = node.subscribe<sensor_msgs::JointState>("joint_states", 10, std::bind(&RobotStatePublisher::jointStateCallback, this, std::placeholders::_1));
    globalTFService = node.advertiseService<std_msgs::Void, geometry_msgs::TF>("global_tf", std::bind(&RobotStatePublisher::globalTFCallback, this, std::placeholders::_1, std::placeholders::_2));
    std::ifstream file(jrdfPath);
    auto json = kineval::JsonParser::parse(file);
    kt = kineval::KinematicTree::fromJson(json->as_object());
}

void RobotStatePublisher::run()
{
    // TCPClient client(URI("0.0.0.0", 9000), false);
    // client.connect();

    node.spin(true);

    const int periodNs = (1 / (float)hz) * 1000000000;

    int64_t start_time = getTimeNano();
    while (node.ok())
    {
        if (getTimeNano() - start_time >= periodNs)
        {
            if (jointStateSub->getNumPublishers() == 0)
            {
                mros::Console::log(mros::LogLevel::DEBUG, "No publishers for joint_states. Stopping interpolation.");
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
    mros::ArgParser::init("robot_state_publisher", "Publishes interpolated transforms based on joint states");

    mros::ArgParser::addArg({"jrdf", "Path to JRDF file", '1'});

    mros::ArgParser::addOpt({"ip", "i", "Core IP address", "0.0.0.0", "", '1'});
    mros::ArgParser::addOpt({"freq", "f", "Publish rate in Hz", "25", "", '1'});
    mros::ArgParser::addOpt({"delta-time", "d", "Time threshold to stop interpolation", "0.1", "", '1'});

    mros::ArgParser::parse(argc, argv);

    std::string jrdfPath = mros::ArgParser::getArg("jrdf")[0];

    URI uri;
    uri.ip = mros::ArgParser::getOpt("ip")[0];
    uri.port = MEDIATOR_PORT_NUM;

    int freq = std::stoi(mros::ArgParser::getOpt("freq")[0]);

    float timeThreshold = std::stof(mros::ArgParser::getOpt("delta-time")[0]);


    RobotStatePublisher RSP(uri, jrdfPath, freq, timeThreshold);
    mros::Console::setLevel(mros::LogLevel::DEBUG);
    RSP.run();

    return 0;
}