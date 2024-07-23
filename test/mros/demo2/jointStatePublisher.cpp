#include "mros/node.hpp"
#include "mros/subscriber.hpp"
#include "mros/optparse.hpp"
#include "utils.hpp"

#include "messages/sensor_msgs/jointState.hpp"

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "linalg/quaternion.hpp"

#include <iostream>
#include <string>
#include <mutex>

struct JointStateObj
{
    double time;
    double position;
    double velocity;
    double effort;
    JointStateObj interpolate(const JointStateObj &next, double targetTime, float timeThreshold) const
    {
        // If the target time is too far ahead of the current time, just return the latest joint state
        if (targetTime - next.time > timeThreshold)
        {
            return next;
        }

        double ratio = (targetTime - time) / (next.time - time);
        JointStateObj result;
        result.time = targetTime;
        result.position = position + ratio * (next.position - position);
        result.velocity = velocity + ratio * (next.velocity - velocity);
        result.effort = effort + ratio * (next.effort - effort);
        return result;
    }
};

class JointStatePublisher
{
public:
    JointStatePublisher(const URI &uri, const std::vector<std::string> &jointStateTopics, int hz, float timeThreshold);

    void run();

private:
    void jointStateCallback(const sensor_msgs::JointState &msg);

    int hz;
    float timeThreshold;
    mros::Node node;
    std::vector<std::shared_ptr<mros::Subscriber>> jointStateSubs;
    std::shared_ptr<mros::Publisher> jointStatePub;

    std::map<std::string, std::pair<JointStateObj, JointStateObj>> jointStateMap;
    sensor_msgs::JointState currJointState;
};

void JointStatePublisher::jointStateCallback(const sensor_msgs::JointState &msg)
{
    // double time = msg.header.stamp.sec.data + msg.header.stamp.nsec.data / 1000000000.0;
    double time = getTimeNano() / 1000000000.0;
    for (int i = 0; i < msg.name.size(); i++)
    {
        std::string name = msg.name[i].data;
        double position = msg.position[i].data;
        double velocity = msg.velocity[i].data;
        double effort = msg.effort[i].data;

        JointStateObj jsObj = {time, position, velocity, effort};
        if (jointStateMap.find(name) == jointStateMap.end())
        {
            currJointState.name.push_back(name);
            currJointState.position.push_back(position);
            currJointState.velocity.push_back(velocity);
            currJointState.effort.push_back(effort);
            jointStateMap[name] = {jsObj, jsObj};
        }
        else
        {
            jointStateMap[name].first = jointStateMap[name].second;
            jointStateMap[name].second = jsObj;
        }
    }
}

JointStatePublisher::JointStatePublisher(const URI &uri, const std::vector<std::string> &jointStateTopics, int hz, float timeThreshold)
    : node("joint_state_publisher", uri), hz(hz), timeThreshold(timeThreshold)
{

    jointStatePub = node.advertise<sensor_msgs::JointState>("joint_states", 10);

    for (std::string topic : jointStateTopics)
    {
        auto jointStateSub = node.subscribe<sensor_msgs::JointState>(topic, 10, std::bind(&JointStatePublisher::jointStateCallback, this, std::placeholders::_1));
        jointStateSubs.push_back(jointStateSub);
    }
}

void JointStatePublisher::run()
{
    const int periodNs = (1 / (float)hz) * 1000000000;

    int64_t start_time = getTimeNano();
    while (node.ok())
    {
        if (getTimeNano() - start_time >= periodNs)
        {

            std_msgs::Time time = std_msgs::Time::getTimeNow();
            currJointState.header.stamp = time;
            double targetTime = time.sec.data + time.nsec.data / 1000000000.0;

            for (int i = 0; i < currJointState.name.size(); ++i)
            {
                std::string name = currJointState.name[i].data;
                JointStateObj interpolated = jointStateMap[name].first.interpolate(jointStateMap[name].second, targetTime, timeThreshold);
                currJointState.position[i] = interpolated.position;
                currJointState.velocity[i] = interpolated.velocity;
                currJointState.effort[i] = interpolated.effort;
            }

            jointStatePub->publish(currJointState);
            start_time = getTimeNano();
        }
        node.spinOnce();
    }
}

int main(int argc, char **argv)
{

    mros::OptionParser::init("joint_state_publisher", "Publishes interpolated joint states");
    mros::OptionParser::addOption({"topics", "t", "Joint state topics", "", "", '+', true});
    mros::OptionParser::addOption({"ip", "i", "Core IP address", "0.0.0.0", "", '1', false});
    mros::OptionParser::addOption({"freq", "f", "Publish rate in Hz", "25", "", '1', false});
    mros::OptionParser::addOption({"delta-time", "d", "Time threshold to stop interpolation", "0.1", "", '1', false});

    mros::OptionParser::parse(argc, argv);

    URI uri;
    uri.ip = mros::OptionParser::getOption("ip")[0];
    uri.port = MEDIATOR_PORT_NUM;
    int freq = std::stoi(mros::OptionParser::getOption("freq")[0]);
    float timeThreshold = std::stof(mros::OptionParser::getOption("delta-time")[0]);
    std::vector<std::string> topics = mros::OptionParser::getOption("topics");

    JointStatePublisher JSP(uri, topics, freq, timeThreshold);
    JSP.run();

    return 0;
}