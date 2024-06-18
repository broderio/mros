#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

#include "jrdf/json.hpp"

struct RobotJoint {
    std::string name;
    std::string type;
    float axis[3];
    float xyz[3];
    float rpy[3];
    float limits[2];
    std::string parentLink;
    std::string childLink;
};

struct RobotLink {
    std::string name;
    float xyz[3];
    float rpy[3];
    float size[3];
    float color[4]; // [r, g, b, a
    std::string parentJoint;
    std::vector<std::string> childJoints;
};

struct RobotModel {
    std::string name;
    std::map<std::string, RobotJoint> joints;
    std::map<std::string, RobotLink> links;
};

RobotModel getRobotModel(const json::JsonObject& json);

#endif // ROBOT_MODEL_HPP