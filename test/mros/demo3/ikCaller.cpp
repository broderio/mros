#include <iostream>
#include <string>

#include "utils.hpp"

#include "messages/geometry_msgs/transform.hpp"
#include "messages/sensor_msgs/jointState.hpp"

#include "mros/utils/argParser.hpp"
#include "mros/core/node.hpp"
#include "mros/core/service.hpp"

#include "kineval/json.hpp"
#include "kineval/tree.hpp"

using namespace mros;

int main(int argc, char **argv)
{
    ArgParser::init("ik_service_node", "Solves the inverse kinematics problem for a given robot");
    ArgParser::addArg({"jrdf", "The json robot description file", '1'});
    ArgParser::addOpt({"ip", "i", "Core IP address", "0.0.0.0", "", '1'});

    ArgParser::parse(argc, argv);

    std::ifstream file(ArgParser::getArg("jrdf")[0]);
    kineval::JsonObject json = kineval::JsonParser::parse(file)->as_object();
    kineval::KinematicTree kt = kineval::KinematicTree::fromJson(json);

    URI uri;
    uri.ip = ArgParser::getOpt("ip")[0];
    uri.port = MEDIATOR_PORT_NUM;

    Node &node = Node::getInstance();
    node.init("service_node", uri);

    Console::setLevel(LogLevel::DEBUG);

    node.spin(false);

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

        Console::log(LogLevel::DEBUG, "Sending request ...");
        if (node.callService("ik_service", req, res))
        {
            Console::log(LogLevel::DEBUG, "IK successful");
            std::cout << "End Effector Goal:\n"
                      << goal.getMatrix() << std::endl;
            std::cout << "\nJoint States: \n"
                      << res << std::endl;
        }
        else
        {
            Console::log(LogLevel::DEBUG, "IK unsuccessful");
        }
    }

    return 0;
}