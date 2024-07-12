#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

#include "jrdf/json.hpp"

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"

#include "messages/geometry_msgs/transform.hpp"

enum JointType {
    REVOLUTE,
    PRISMATIC
};

struct BodyState {
    float xyz[3];
    float rpy[3];
};

class RobotJoint {
friend class RobotModel;
public:
    std::string getName();
    JointType getType();
    linalg::Vector getAxis();
    float getPosition();
    void setPosition(float position); // Will update the kinematic chain that this action affects
    BodyState getOrigin();
    std::vector<float> getLimits();

private:
    std::string name;
    JointType type;
    float axis[3];
    float position;
    BodyState origin; // Relative to parent link's parent joint
    float limits[2];
    linalg::Matrix transform; // Transform of joint frame with respect to the world frame
    std::string parentLink;
    std::string childLink;
};

class RobotLink {
friend class RobotModel;
public:
    std::string getName();
    bool isEndeffector();
    std::string getParentJoint();
    std::vector<std::string> getChildJoints();

    BodyState getGlobalPose();
    BodyState getRelativePose();
    BodyState getRelativePose(const RobotLink& link);

private:
    std::string name;
    bool isEndeffector_;
    std::string parentJoint;
    std::vector<std::string> childJoints;
};

class RobotModel {
public:
    RobotModel(const json::JsonObject& json);
    RobotModel(const std::string& jsonStr);
    RobotModel(const std::ifstream& jsonFile);

    RobotLink getLink(const std::string& link);
    std::vector<std::string> getLinkNames();

    RobotJoint getJoint(const std::string& joint);
    std::vector<std::string> getJointNames();
    void setJointPositions(const std::vector<std::string>& names, const std::vector<float> positions);

    geometry_msgs::TF getTransformChain(); 

private:
    std::string name;
    std::string baseLink;
    std::vector<std::string> endeffectorLinks;

    std::map<std::string, RobotJoint> joints;
    std::map<std::string, RobotLink> links;
};

#endif // ROBOT_MODEL_HPP