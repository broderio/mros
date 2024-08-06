#include "mros/core/node.hpp"
#include "mros/core/publisher.hpp"
#include "mros/utils/argParser.hpp"
#include "utils.hpp"

#include "messages/geometry_msgs/transform.hpp"
#include "messages/sensor_msgs/jointState.hpp"

#include "kineval/json.hpp"
#include "kineval/tree.hpp"

#include <iostream>
#include <string>

int main(int argc, char **argv) 
{
    mros::ArgParser::init("ik_service_node", "Solves the inverse kinematics problem for a given robot"); 
    mros::ArgParser::addArg({"jrdf", "The json robot description file", '1'});
    mros::ArgParser::addOpt({"ip", "i", "Core IP address", "0.0.0.0", "", '1'});

    mros::ArgParser::parse(argc, argv);

    std::ifstream file(mros::ArgParser::getArg("jrdf")[0]);
    kineval::JsonObject json = kineval::JsonParser::parse(file)->as_object();
    kineval::KinematicTree kt = kineval::KinematicTree::fromJson(json);

    URI uri;
    uri.ip = mros::ArgParser::getOpt("ip")[0];
    uri.port = MEDIATOR_PORT_NUM;

    mros::Node node("service_node", uri);

    mros::Console::setLevel(mros::LogLevel::DEBUG);

    node.spin(true);

    while (node.ok())
    {

        for (auto &name : kt.getJointNames())
        {
            double l, u;
            kt.getJoint(name).getLimits(l, u);
            std::cout << "Input state for \"" << name << "\"" << " [" << l << ", " << u << "]" << std::endl;
            double q;
            std::cin >> q;
            kt.setJointState(name, q);
        }

        kineval::Transform goal;
        goal.setTranslation(kt.getGlobalLinkTransform(kt.getEndEffector()).getTranslation());

        geometry_msgs::Transform req(goal.getTranslation().toMsg(), goal.getRotation().toMsg());
        sensor_msgs::JointState res;

        mros::Console::log(mros::LogLevel::DEBUG, "Sending request ...");
        if (node.callService("ik_service", req, res))
        {
            mros::Console::log(mros::LogLevel::DEBUG, "IK successful");
            std::cout << "End Effector Goal:\n" << goal.getMatrix() << std::endl;
            std::cout << "\nJoint States: \n" << res << std::endl;
        }
        else
        {
            mros::Console::log(mros::LogLevel::DEBUG, "IK unsuccessful");
        }
    }

    return 0;
}