#include <iostream>
#include <string>

#include "utils.hpp"

#include "messages/geometry_msgs/transform.hpp"
#include "messages/sensor_msgs/jointState.hpp"

#include "mros/utils/console.hpp"
#include "mros/utils/argParser.hpp"
#include "mros/core/node.hpp"
#include "mros/core/service.hpp"

#include "kineval/json.hpp"
#include "kineval/tree.hpp"

using namespace mros;

int maxIter;
float tol;
kineval::KinematicTree kt;

void serviceCallback(const geometry_msgs::Transform &req, sensor_msgs::JointState &res)
{
    kineval::Transform goal;
    goal.setTranslation(req.translation);
    goal.setRotation(req.rotation);

    int iter = maxIter;
    if (kt.solveIK(res, goal, tol, iter))
    {
        Console::log(LogLevel::INFO, "IK successful in " + std::to_string(iter) + " iterations");
    }
    else
    {
        Console::log(LogLevel::INFO, "IK unsuccessful");
    }
}

int main(int argc, char **argv)
{
    ArgParser::init("ik_service_node", "Solves the inverse kinematics problem for a given robot");
    ArgParser::addArg({"jrdf", "The json robot description file", '1'});

    ArgParser::addOpt({"ip", "i", "Core IP address", "0.0.0.0", "", '1'});
    ArgParser::addOpt({"max-iter", "m", "Maximum number of iterations for the IK solver", "10000", "", '1'});
    ArgParser::addOpt({"tol", "t", "Tolerance for the IK solver", "0.005", "", '1'});

    ArgParser::parse(argc, argv);

    URI uri;
    uri.ip = ArgParser::getOpt("ip")[0];
    uri.port = MEDIATOR_PORT_NUM;

    maxIter = std::stoi(ArgParser::getOpt("max-iter")[0]);
    tol = std::stof(ArgParser::getOpt("tol")[0]);

    std::ifstream file(ArgParser::getArg("jrdf")[0]);
    kineval::JsonObject json = kineval::JsonParser::parse(file)->as_object();
    kt = kineval::KinematicTree::fromJson(json);

    Node &node = Node::getInstance();
    node.init("ik_service_node", uri);

    Console::setLevel(LogLevel::DEBUG);

    std::shared_ptr<Service> srv = node.advertiseService<geometry_msgs::Transform, sensor_msgs::JointState>("ik_service", &serviceCallback);

    if (srv == nullptr)
    {
        std::cout << "Failed to advertise service" << std::endl;
        return 1;
    }

    node.spin();

    return 0;
}