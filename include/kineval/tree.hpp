#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <stack>

#include "kineval/json.hpp"
#include "kineval/transform.hpp"
#include "kineval/joint.hpp"
#include "kineval/link.hpp"

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "linalg/rotation.hpp"

#include "messages/geometry_msgs/transform.hpp"
#include "messages/sensor_msgs/jointState.hpp"

namespace kineval
{

    class KinematicTree
    {
    public:
        KinematicTree();
        KinematicTree(const std::string &name);
        KinematicTree(const KinematicTree &t);
        KinematicTree &operator=(const KinematicTree &t);

        static KinematicTree fromJson(const JsonObject &json);

        std::string getName() const;

        Joint getJoint(const std::string &name) const;
        Link getLink(const std::string &name) const;

        void addJoint(const std::string &name, const std::string &parent, const std::string &child, const Joint &joint);
        void addLink(const std::string &name, const Link &link);

        void setRoot(const std::string &name);
        std::string getRoot() const;

        void setEndEffector(const std::string &name);
        std::string getEndEffector() const;

        std::vector<std::string> getJointNames() const;
        std::vector<std::string> getLinkNames() const;

        const std::string &getParentJoint(const std::string &name) const;
        const std::string &getParentLink(const std::string &name) const;

        const std::vector<std::string> &getChildJoints(const std::string &name) const;
        const std::string &getChildLink(const std::string &name) const;

        void setJointState(const std::string &name, double q);
        void setJointStates(const sensor_msgs::JointState &q);
        void setJointStates(const std::vector<double> &q);
        void setJointStates(const std::map<std::string, double> &q);

        double getJointState(const std::string &name) const;

        std::vector<double> getJointStates() const;
        std::map<std::string, double> getJointStatesMap() const;
        sensor_msgs::JointState getJointStateMsg() const;

        Transform getJointTransform(const std::string &name) const;
        Transform getLinkTransform(const std::string &name) const;

        Transform getGlobalJointTransform(const std::string &name) const;
        Transform getGlobalLinkTransform(const std::string &name) const;

        std::map<std::string, Transform> getJointTransforms() const;
        std::map<std::string, Transform> getLinkTransforms() const;

        void getGlobalTransforms(std::map<std::string, Transform> &jointTransforms, std::map<std::string, Transform> &linkTransforms) const;

        geometry_msgs::TF TF() const;
        geometry_msgs::TF globalTF() const;

        linalg::Matrix getJacobian(const std::string &endEffector) const;
        bool solveIK(sensor_msgs::JointState &js, const Transform &endEffectorGoal, double tol, int &maxIter);

    private:

        void getGlobalJointTransform(const std::string &name, const Transform &transform, geometry_msgs::TF &tf, std_msgs::Header header) const;
        void getGlobalLinkTransform(const std::string &name, const Transform &transform, geometry_msgs::TF &tf, std_msgs::Header header) const;

        void getGlobalJointTransform(const std::string &name, const Transform &transform, std::map<std::string, Transform> &jointTransforms, std::map<std::string, Transform> &linkTransforms) const;
        void getGlobalLinkTransform(const std::string &name, const Transform &transform, std::map<std::string, Transform> &jointTransforms, std::map<std::string, Transform> &linkTransforms) const;

        struct LinkNode;

        struct JointNode
        {
            Joint joint;
            std::string name;
            std::string parent;
            std::string child;
        };

        struct LinkNode
        {
            Link link;
            std::string name;
            std::string parent;
            std::vector<std::string> children;
        };

        std::set<std::string> jointNames;
        std::set<std::string> linkNames;

        std::map<std::string, JointNode> joints;
        std::map<std::string, LinkNode> links;

        std::string name;
        std::string root;
        std::string endEffector;
    };

} // namespace kineval