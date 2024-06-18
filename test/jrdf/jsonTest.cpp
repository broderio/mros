#include <iostream>
#include <string>
#include <ostream>
#include <fstream>

#include "jrdf/json.hpp"
#include "jrdf/robotModel.hpp"

using namespace json;

int main() {
    std::ifstream file("/Users/broderio/Repositories/simple_pubsub/robots/simpleBot.json");
    JsonObject json = JsonParser::parse(file)->as_object();

    RobotModel robot = getRobotModel(json); 

    std::cout << "Robot name: " << robot.name << std::endl;
    std::cout << "Joints: " << std::endl;
    for (auto& j : robot.joints) {
        std::cout << "  " << j.first << std::endl;
        std::cout << "    Type: " << j.second.type << std::endl;
        std::cout << "    Axis: " << j.second.axis[0] << ", " << j.second.axis[1] << ", " << j.second.axis[2] << std::endl;
        std::cout << "    XYZ: " << j.second.xyz[0] << ", " << j.second.xyz[1] << ", " << j.second.xyz[2] << std::endl;
        std::cout << "    RPY: " << j.second.rpy[0] << ", " << j.second.rpy[1] << ", " << j.second.rpy[2] << std::endl;
        std::cout << "    Limits: " << j.second.limits[0] << ", " << j.second.limits[1] << std::endl;
        std::cout << "    Parent link: " << j.second.parentLink << std::endl;
        std::cout << "    Child link: " << j.second.childLink << std::endl;
        std::cout << std::endl;
    }
    std::cout << "Links: " << std::endl;
    for (auto& l : robot.links) {
        std::cout << "  " << l.first << std::endl;
        std::cout << "    XYZ: " << l.second.xyz[0] << ", " << l.second.xyz[1] << ", " << l.second.xyz[2] << std::endl;
        std::cout << "    RPY: " << l.second.rpy[0] << ", " << l.second.rpy[1] << ", " << l.second.rpy[2] << std::endl;
        std::cout << "    Size: " << l.second.size[0] << ", " << l.second.size[1] << ", " << l.second.size[2] << std::endl;
        std::cout << "    Color: " << l.second.color[0] << ", " << l.second.color[1] << ", " << l.second.color[2] << ", " << l.second.color[3] << std::endl;
        std::cout << "    Parent joint: " << l.second.parentJoint << std::endl;
        std::cout << "    Child joints: ";
        for (auto& c : l.second.childJoints) {
            std::cout << c;
            if (&c != &l.second.childJoints.back()) {
                std::cout << ", ";
            }
        }
        std::cout << "\n\n";
    }

}