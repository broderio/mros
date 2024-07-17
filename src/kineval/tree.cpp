#include "kineval/tree.hpp"

namespace kineval
{

    KinematicTree::KinematicTree()
    {
    }

    KinematicTree::KinematicTree(const std::string &name)
        : name(name)
    {
    }

    KinematicTree KinematicTree::fromJson(const JsonObject &json)
    {
        bool hasName = json.find("name") != json.end();
        bool hasEndeffector = json.find("endeffector") != json.end();
        bool hasJoints = json.find("joints") != json.end();
        bool hasLinks = json.find("links") != json.end();

        if (!hasName || !hasEndeffector || !hasJoints || !hasLinks)
        {
            throw std::runtime_error("Invalid robot model JSON");
        }

        std::string name = json.at("name")->as_string();
        KinematicTree kt(name);

        kt.endEffector = json.at("endeffector")->as_string();

        // Convert json joint objects to JointNode objects
        JsonObject joints = json.at("joints")->as_object();
        for (auto &j : joints)
        {
            std::string name = j.first;

            JsonObject jointObj = j.second->as_object();

            bool hasType = jointObj.find("type") != jointObj.end();
            bool hasAxis = jointObj.find("axis") != jointObj.end();
            bool hasOrigin = jointObj.find("origin") != jointObj.end();
            bool hasParentLink = jointObj.find("parentLink") != jointObj.end();

            if (!hasType || !hasAxis || !hasOrigin || !hasParentLink)
            {
                throw std::runtime_error("Invalid joint JSON");
            }

            JsonObject origin = jointObj["origin"]->as_object();
            bool hasXYZ = origin.find("xyz") != origin.end();
            bool hasRPY = origin.find("rpy") != origin.end();

            if (!hasXYZ || !hasRPY)
            {
                throw std::runtime_error("Invalid origin JSON");
            }

            std::string parentLink = jointObj["parentLink"]->as_string();

            std::string typeStr = jointObj["type"]->as_string();
            JointType type;
            if (typeStr == "revolute")
            {
                type = JointType::REVOLUTE;
            }
            else if (typeStr == "prismatic")
            {
                type = JointType::PRISMATIC;
            }
            else
            {
                throw std::runtime_error("Invalid joint type");
            }

            JsonArray axis = jointObj["axis"]->as_array();
            linalg::Vector a(3);
            a.at(0) = axis[0]->as_float();
            a.at(1) = axis[1]->as_float();
            a.at(2) = axis[2]->as_float();

            linalg::Vector o(6);
            JsonArray xyz = origin["xyz"]->as_array();
            o.at(0) = xyz[0]->as_float();
            o.at(1) = xyz[1]->as_float();
            o.at(2) = xyz[2]->as_float();

            JsonArray rpy = origin["rpy"]->as_array();
            o.at(3) = rpy[0]->as_float();
            o.at(4) = rpy[1]->as_float();
            o.at(5) = rpy[2]->as_float();

            Joint joint(a, o, type);
            kt.addJoint(name, parentLink, joint);
        }

        JsonObject links = json.at("links")->as_object();
        for (auto &l : links)
        {
            std::string name = l.first;

            JsonObject linkObj = l.second->as_object();

            bool hasOrigin = linkObj.find("origin") != linkObj.end();
            bool hasParentJoint = linkObj.find("parentJoint") != linkObj.end();

            if (!hasOrigin)
            {
                throw std::runtime_error("Invalid link JSON");
            }

            if (!hasParentJoint)
            {
                kt.root = name;
            }

            JsonObject origin = linkObj["origin"]->as_object();
            bool hasXYZ = origin.find("xyz") != origin.end();
            bool hasRPY = origin.find("rpy") != origin.end();

            if (!hasXYZ || !hasRPY)
            {
                throw std::runtime_error("Invalid origin JSON");
            }

            std::string parentJoint = (hasParentJoint) ? linkObj["parentJoint"]->as_string() : "";

            linalg::Vector o(6);
            JsonArray xyz = origin["xyz"]->as_array();
            o.at(0) = xyz[0]->as_float();
            o.at(1) = xyz[1]->as_float();
            o.at(2) = xyz[2]->as_float();

            JsonArray rpy = origin["rpy"]->as_array();
            o.at(3) = rpy[0]->as_float();
            o.at(4) = rpy[1]->as_float();
            o.at(5) = rpy[2]->as_float();

            Link link(o);
            kt.addLink(name, parentJoint, link);
        }
        return kt;
    }

    KinematicTree::KinematicTree(const KinematicTree &t)
    {
        jointNames = t.jointNames;
        linkNames = t.linkNames;
        joints = t.joints;
        links = t.links;
        root = t.root;
    }

    KinematicTree &KinematicTree::operator=(const KinematicTree &t)
    {
        if (this == &t)
        {
            return *this;
        }

        jointNames = t.jointNames;
        linkNames = t.linkNames;
        joints = t.joints;
        links = t.links;
        root = t.root;

        return *this;
    }

    std::string KinematicTree::getName() const 
    {
        return name;
    }

    void KinematicTree::addJoint(const std::string &name, const std::string &parent, const Joint &joint)
    {
        // If it doesn't exist, add it
        if (jointNames.find(name) == jointNames.end())
        {
            jointNames.insert(name);
            JointNode j = {joint, name, parent, ""};
            joints[name] = j;
        }
        // If it does exist, update it (this joint was already added as the parent of a link)
        else
        {
            joints[name].joint = joint;
            joints[name].parent = parent;
        }

        // If the parent link doesn't exist, add it
        if (linkNames.find(parent) == linkNames.end())
        {
            linkNames.insert(parent);
            LinkNode l;
            l.name = parent;
            l.parent = "";
            links[parent] = l;
        }

        links[parent].children.push_back(name);
    }

    void KinematicTree::addLink(const std::string &name, const std::string &parent, const Link &link)
    {
        // If it doesn't exist, add it
        if (linkNames.find(name) == linkNames.end())
        {
            linkNames.insert(name);
            LinkNode l = {link, name, parent, {}};
            links[name] = l;
        }
        // If it does exist, update it (this link was already added as the parent of a joint)
        else
        {
            links[name].link = link;
            links[name].parent = parent;
        }

        // If the parent joint doesn't exist, add it
        if (jointNames.find(parent) == jointNames.end())
        {
            jointNames.insert(parent);
            JointNode j;
            j.name = parent;
            j.parent = "";
            joints[parent] = j;
        }

        joints[parent].child = name;
    }

    void KinematicTree::setRoot(const std::string &name)
    {
        if (linkNames.find(name) == linkNames.end())
        {
            throw std::runtime_error("Link not found");
        }

        root = name;
    }

    std::string KinematicTree::getRoot() const
    {
        return root;
    }

    void KinematicTree::setEndEffector(const std::string &name)
    {
        if (linkNames.find(name) == linkNames.end())
        {
            throw std::runtime_error("Link not found");
        }

        endEffector = name;
    }

    std::string KinematicTree::getEndEffector() const
    {
        return endEffector;
    }

    const std::set<std::string> &KinematicTree::getJointNames() const
    {
        return jointNames;
    }

    const std::set<std::string> &KinematicTree::getLinkNames() const
    {
        return linkNames;
    }

    const std::string &KinematicTree::getParentJoint(const std::string &name) const
    {
        if (joints.find(name) == joints.end())
        {
            throw std::runtime_error("Joint not found");
        }

        return joints.at(name).parent;
    }

    const std::string &KinematicTree::getParentLink(const std::string &name) const
    {
        if (links.find(name) == links.end())
        {
            throw std::runtime_error("Link not found");
        }

        return links.at(name).parent;
    }

    const std::vector<std::string> &KinematicTree::getChildJoints(const std::string &name) const
    {
        if (linkNames.find(name) == linkNames.end())
        {
            throw std::runtime_error("Link not found");
        }

        return links.at(name).children;
    }

    const std::string &KinematicTree::getChildLink(const std::string &name) const
    {
        if (jointNames.find(name) == jointNames.end())
        {
            throw std::runtime_error("Joint not found");
        }

        return joints.at(name).child;
    }

    void KinematicTree::setJointState(const std::string &name, float q)
    {
        if (joints.find(name) == joints.end())
        {
            throw std::runtime_error("Joint not found");
        }

        joints.at(name).joint.q = q;
    }

    float KinematicTree::getJointState(const std::string &name) const
    {
        if (joints.find(name) == joints.end())
        {
            throw std::runtime_error("Joint not found");
        }

        return joints.at(name).joint.q;
    }

    Transform KinematicTree::getJointTransform(const std::string &name) const
    {
        if (joints.find(name) == joints.end())
        {
            throw std::runtime_error("Joint not found");
        }

        return joints.at(name).joint.getTransform();
    }

    Transform KinematicTree::getLinkTransform(const std::string &name) const
    {
        if (links.find(name) == links.end())
        {
            throw std::runtime_error("Link not found");
        }

        return links.at(name).link.getTransform();
    }

    Transform KinematicTree::getGlobalJointTransform(const std::string &name) const
    {
        if (joints.find(name) == joints.end())
        {
            throw std::runtime_error("Joint not found");
        }

        Transform T = joints.at(name).joint.getTransform();
        std::string parent = joints.at(name).parent;

        T.applyTransform(getGlobalLinkTransform(parent));
        return T;
    }

    Transform KinematicTree::getGlobalLinkTransform(const std::string &name) const
    {
        if (links.find(name) == links.end())
        {
            throw std::runtime_error("Link not found");
        }

        if (name == root)
        {
            return links.at(name).link.getTransform();
        }

        Transform T = links.at(name).link.getTransform();
        std::string parent = links.at(name).parent;

        T.applyTransform(getGlobalJointTransform(parent));
        return T;
    }

    geometry_msgs::TF KinematicTree::TF() const
    {
        std_msgs::Time stamp;
        stamp.sec = getTimeNano() / 1000000000;
        stamp.nsec = getTimeNano() % 1000000000;

        geometry_msgs::TF tf;

        int seq = 0;
        for (const auto &j : joints)
        {
            geometry_msgs::TransformStamped t;
            t.header.stamp = stamp;
            t.header.seq = seq++;
            t.header.frame_id = j.second.name;
            t.child_frame_id = j.second.child;
            t.transform.translation = j.second.joint.getTransform().getTranslation().toMsg();
            t.transform.rotation = j.second.joint.getTransform().getRotation().getQuaternion().toMsg();
        }

        return tf;
    }

} // namespace kineval