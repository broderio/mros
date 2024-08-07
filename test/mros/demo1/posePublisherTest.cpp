#include <iostream>
#include <string>

#include "utils.hpp"

#include "messages/geometry_msgs/twist.hpp"
#include "messages/geometry_msgs/pose.hpp"

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "linalg/quaternion.hpp"

#include "mros/common.hpp"
#include "mros/utils/console.hpp"
#include "mros/core/node.hpp"
#include "mros/core/subscriber.hpp"

using namespace mros;
using std::placeholders::_1;

class PosePublisher
{
public:
    PosePublisher(URI uri);

    void run();

private:
    void twistCallback(const geometry_msgs::TwistStamped &msg);

    std::shared_ptr<Publisher> posePub;
    std::shared_ptr<Subscriber> twistSub;

    geometry_msgs::PoseStamped currPose;
    linalg::Vector rpy;
};

void PosePublisher::twistCallback(const geometry_msgs::TwistStamped &msg)
{
    // Rotate velocity vector by current orientation
    linalg::Matrix rotMat = linalg::Quaternion::toRotationMatrix(linalg::Quaternion(currPose.pose.orientation));
    linalg::Vector vel(3, 0);
    vel.at(0) = msg.twist.linear.x.data;
    vel.at(1) = msg.twist.linear.y.data;
    vel.at(2) = msg.twist.linear.z.data;
    linalg::Vector velGlobalFrame = linalg::Matrix::multiply(rotMat, vel);

    const float dt = 0.1;
    currPose.pose.position.x.data += velGlobalFrame.at(0) * dt;
    currPose.pose.position.y.data += velGlobalFrame.at(1) * dt;
    currPose.pose.position.z.data += velGlobalFrame.at(2) * dt;

    rpy.at(0) += msg.twist.angular.x.data * dt;
    rpy.at(1) += msg.twist.angular.y.data * dt;
    rpy.at(2) += msg.twist.angular.z.data * dt;

    currPose.pose.orientation = linalg::Quaternion::fromRPY(rpy.at(0), rpy.at(1), rpy.at(2)).toMsg();

    posePub->publish(currPose);
}

PosePublisher::PosePublisher(URI uri)
{
    Node &node = Node::getInstance();
    node.init("pose_publisher", uri);
    Console::setLevel(LogLevel::DEBUG);
    posePub = node.advertise<geometry_msgs::PoseStamped>("pose_topic", 10);
    twistSub = node.subscribe<geometry_msgs::TwistStamped>("twist_topic", 10, std::bind(&PosePublisher::twistCallback, this, _1));
}

void PosePublisher::run()
{
    if (posePub == nullptr || twistSub == nullptr)
    {
        std::cout << "Failed to advertise topic or subscribe to topic" << std::endl;
        return;
    }

    currPose.header.frame_id = "world";
    currPose.pose.position.x.data = 0;
    currPose.pose.position.y.data = 0;
    currPose.pose.position.z.data = 0;
    currPose.pose.orientation = linalg::Quaternion::fromRPY(0, 0, 0).toMsg();

    rpy = linalg::Vector(3, 0);

    Node &node = Node::getInstance();
    node.spin();
}

int main()
{
    PosePublisher posePub(URI(getLocalIP(), MEDIATOR_PORT_NUM));
    posePub.run();

    return 0;
}