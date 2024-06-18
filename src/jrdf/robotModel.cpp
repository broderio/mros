#include "jrdf/robotModel.hpp"

using namespace json;

RobotModel getRobotModel(const json::JsonObject& json) {
    RobotModel robot;

    bool hasName = json.find("name") != json.end();
    bool hasJoints = json.find("joints") != json.end();
    bool hasLinks = json.find("links") != json.end();

    if (!hasName || !hasJoints || !hasLinks) {
        throw std::runtime_error("Invalid robot model JSON");
    }

    robot.name = json.at("name")->as_string();

    // Convert json joint objects to RobotJoint objects
    RobotJoint joint;
    JsonObject joints = json.at("joints")->as_object();
    for (auto& j : joints) {
        joint.name = j.first;

        JsonObject jointObj = j.second->as_object();

        bool hasType = jointObj.find("type") != jointObj.end();
        bool hasAxis = jointObj.find("axis") != jointObj.end();
        bool hasXYZ = jointObj.find("xyz") != jointObj.end();
        bool hasRPY = jointObj.find("rpy") != jointObj.end();
        bool hasParentLink = jointObj.find("parentLink") != jointObj.end();
        bool hasChildLink = jointObj.find("childLink") != jointObj.end();

        if (!hasType || !hasAxis || !hasXYZ || !hasRPY || !hasParentLink || !hasChildLink) {
            throw std::runtime_error("Invalid joint JSON");
        }

        joint.type = jointObj["type"]->as_string();

        JsonArray axis = jointObj["axis"]->as_array();
        joint.axis[0] = axis[0]->as_float();
        joint.axis[1] = axis[1]->as_float();
        joint.axis[2] = axis[2]->as_float();

        JsonArray xyz = jointObj["xyz"]->as_array();
        joint.xyz[0] = xyz[0]->as_float();
        joint.xyz[1] = xyz[1]->as_float();
        joint.xyz[2] = xyz[2]->as_float();

        JsonArray rpy = jointObj["rpy"]->as_array();
        joint.rpy[0] = rpy[0]->as_float();
        joint.rpy[1] = rpy[1]->as_float();
        joint.rpy[2] = rpy[2]->as_float();

        joint.parentLink = jointObj["parentLink"]->as_string();
        joint.childLink = jointObj["childLink"]->as_string();

        JsonArray limits = jointObj["limits"]->as_array();
        joint.limits[0] = limits[0]->as_float();
        joint.limits[1] = limits[1]->as_float();

        robot.joints[joint.name] = joint;
    }

    RobotLink link;
    JsonObject links = json.at("links")->as_object();
    for (auto& l : links) {
        link.name = l.first;

        JsonObject linkObj = l.second->as_object();

        bool hasXYZ = linkObj.find("xyz") != linkObj.end();
        bool hasRPY = linkObj.find("rpy") != linkObj.end();
        bool hasSize = linkObj.find("size") != linkObj.end();
        bool hasColor = linkObj.find("color") != linkObj.end();

        if (!hasXYZ || !hasRPY || !hasSize) {
            throw std::runtime_error("Invalid link JSON");
        }

        JsonArray xyz = linkObj["xyz"]->as_array();
        link.xyz[0] = xyz[0]->as_float();
        link.xyz[1] = xyz[1]->as_float();
        link.xyz[2] = xyz[2]->as_float();

        JsonArray rpy = linkObj["rpy"]->as_array();
        link.rpy[0] = rpy[0]->as_float();
        link.rpy[1] = rpy[1]->as_float();
        link.rpy[2] = rpy[2]->as_float();

        JsonArray size = linkObj["size"]->as_array();
        link.size[0] = size[0]->as_float();
        link.size[1] = size[1]->as_float();
        link.size[2] = size[2]->as_float();

        if (hasColor) {
            JsonArray color = linkObj["color"]->as_array();
            link.color[0] = color[0]->as_float();
            link.color[1] = color[1]->as_float();
            link.color[2] = color[2]->as_float();
            link.color[3] = color[3]->as_float();
        }
        else {
            link.color[0] = 0.5;
            link.color[1] = 0.5;
            link.color[2] = 0.5;
            link.color[3] = 1.0;
        }

        robot.links[link.name] = link;
    }

    for (auto& j : robot.joints) {
        robot.links[j.second.childLink].parentJoint = j.first;
        robot.links[j.second.parentLink].childJoints.push_back(j.first);
    }

    return robot;
}
