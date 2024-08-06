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

int maxIter;
float tol;
kineval::KinematicTree kt;

void serviceCallback(const geometry_msgs::Transform &req, sensor_msgs::JointState &res) 
{
    kineval::Transform goal;
    goal.setTranslation(req.translation);
    goal.setRotation(req.rotation);

    int iter = maxIter;
    if (kt.solveIK(res, goal, tol, iter)) {
        mros::Console::log(mros::LogLevel::INFO, "IK successful in " + std::to_string(iter) + " iterations");
    } else {
        mros::Console::log(mros::LogLevel::INFO, "IK unsuccessful");
    }
}

int main(int argc, char **argv) 
{
    mros::ArgParser::init("ik_service_node", "Solves the inverse kinematics problem for a given robot"); 
    mros::ArgParser::addArg({"jrdf", "The json robot description file", '1'});

    mros::ArgParser::addOpt({"ip", "i", "Core IP address", "0.0.0.0", "", '1'});
    mros::ArgParser::addOpt({"max-iter", "m", "Maximum number of iterations for the IK solver", "10000", "", '1'});
    mros::ArgParser::addOpt({"tol", "t", "Tolerance for the IK solver", "0.005", "", '1'});

    mros::ArgParser::parse(argc, argv);

    URI uri;
    uri.ip = mros::ArgParser::getOpt("ip")[0];
    uri.port = MEDIATOR_PORT_NUM;

    maxIter = std::stoi(mros::ArgParser::getOpt("max-iter")[0]);
    tol = std::stof(mros::ArgParser::getOpt("tol")[0]);

    std::ifstream file(mros::ArgParser::getArg("jrdf")[0]);
    kineval::JsonObject json = kineval::JsonParser::parse(file)->as_object();
    kt = kineval::KinematicTree::fromJson(json);

    mros::Node node("ik_service_node", uri);

    mros::Console::setLevel(mros::LogLevel::DEBUG);

    std::shared_ptr<mros::Service> srv = node.advertiseService<geometry_msgs::Transform, sensor_msgs::JointState>("ik_service", &serviceCallback);

    if (srv == nullptr) {
        std::cout << "Failed to advertise service" << std::endl;
        return 1;
    }

    node.spin();

    return 0;
}