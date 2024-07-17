#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <set>

#include "kineval/json.hpp"
#include "kineval/transform.hpp"
#include "kineval/joint.hpp"
#include "kineval/link.hpp"

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "linalg/rotation.hpp"

#include "messages/geometry_msgs/transform.hpp"

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

        void addJoint(const std::string &name, const std::string &parent, const Joint &joint);
        void addLink(const std::string &name, const std::string &parent, const Link &link);

        void setRoot(const std::string &name);
        std::string getRoot() const;

        void setEndEffector(const std::string &name);
        std::string getEndEffector() const;

        const std::set<std::string> &getJointNames() const;
        const std::set<std::string> &getLinkNames() const;

        const std::string &getParentJoint(const std::string &name) const;
        const std::string &getParentLink(const std::string &name) const;

        const std::vector<std::string> &getChildJoints(const std::string &name) const;
        const std::string &getChildLink(const std::string &name) const;

        void setJointState(const std::string &name, float q);
        float getJointState(const std::string &name) const;

        Transform getJointTransform(const std::string &name) const;
        Transform getLinkTransform(const std::string &name) const;

        Transform getGlobalJointTransform(const std::string &name) const;
        Transform getGlobalLinkTransform(const std::string &name) const;

        geometry_msgs::TF TF() const;

    private:
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