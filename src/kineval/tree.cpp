#include "kineval/tree.hpp"

#include <random>

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
        // Check for required fields of JRDF
        bool hasName = json.find("name") != json.end();
        bool hasEndeffector = json.find("endeffector") != json.end();
        bool hasBase = json.find("base") != json.end();
        bool hasJoints = json.find("joints") != json.end();
        bool hasLinks = json.find("links") != json.end();

        if (!hasName || !hasEndeffector || !hasJoints || !hasLinks)
        {
            throw std::runtime_error("Invalid robot model JSON");
        }

        // Create Kinematic Tree
        std::string name = json.at("name")->as_string();
        KinematicTree kt(name);

        kt.endEffector = json.at("endeffector")->as_string();
        kt.root = json.at("base")->as_string();

        // Parse JSON joints list into Joint objects
        JsonArray joints = json.at("joints")->as_array();
        for (auto j : joints)
        {
            JsonObject jointObj = j->as_object();

            // Check for required fields of joints
            bool hasName = jointObj.find("name") != jointObj.end();
            bool hasParent = jointObj.find("parent") != jointObj.end();
            bool hasChild = jointObj.find("child") != jointObj.end();
            bool hasType = jointObj.find("type") != jointObj.end();

            if (!hasName || !hasParent || !hasChild || !hasType)
            {
               throw std::runtime_error("Invalid joint JSON");
            }
            std::string jointName = jointObj["name"]->as_string();
            std::string parent = jointObj["parent"]->as_string();
            std::string child = jointObj["child"]->as_string();
            std::string typeStr = jointObj["type"]->as_string();

            // Check for optional origin field
            linalg::Vector o(6, 0);
            bool hasOrigin = jointObj.find("origin") != jointObj.end();
            if (hasOrigin)
            {
                JsonObject origin = jointObj["origin"]->as_object();
                bool hasXYZ = origin.find("xyz") != origin.end();
                bool hasRPY = origin.find("rpy") != origin.end();

                if (!hasXYZ || !hasRPY)
                {
                    throw std::runtime_error("Invalid origin JSON");
                }

                JsonArray xyz = origin["xyz"]->as_array();
                JsonArray rpy = origin["rpy"]->as_array();
                if (xyz.size() != 3 || rpy.size() != 3)
                {
                    throw std::runtime_error("Invalid size for origin array");
                }

                o.at(0) = xyz[0]->as_double();
                o.at(1) = xyz[1]->as_double();
                o.at(2) = xyz[2]->as_double();
                o.at(3) = rpy[0]->as_double();
                o.at(4) = rpy[1]->as_double();
                o.at(5) = rpy[2]->as_double();
            }

            // Parser joint type
            JointType type;
            if (typeStr == "revolute")
            {
                type = JointType::REVOLUTE;
            }
            else if (typeStr == "prismatic")
            {
                type = JointType::PRISMATIC;
            }
            else if (typeStr == "fixed")
            {
                type = JointType::FIXED;
            }
            else if (typeStr == "continuous")
            {
                type = JointType::CONTINUOUS;
            }
            else
            {
                throw std::runtime_error("Invalid joint type");
            }

            // Parse axis
            linalg::Vector a({0, 0, 1});
            if (type != JointType::FIXED)
            {
                bool hasAxis = jointObj.find("axis") != jointObj.end();
                JsonArray axis = jointObj["axis"]->as_array();
                if (axis.size() != 3)
                {
                    throw std::runtime_error("Invalid size for axis array");
                }

                a.at(0) = axis[0]->as_double();
                a.at(1) = axis[1]->as_double();
                a.at(2) = axis[2]->as_double();
            }

            // Create joint
            Joint joint(a, o, type);

            // Check for joint limits
            bool hasLimits = jointObj.find("limits") != jointObj.end();
            if (hasLimits)
            {
                JsonObject limits = jointObj.at("limits")->as_object();

                bool hasUpper = limits.find("upper") != limits.end();
                bool hasLower = limits.find("lower") != limits.end();

                if (!hasUpper || !hasLower)
                {
                    throw std::runtime_error("Invalid limits JSON");
                }

                double lower = limits.at("lower")->as_double();
                double upper = limits.at("upper")->as_double();

                joint.setLimits(lower, upper);
            }

            // Add joint to tree
            kt.addJoint(jointName, parent, child, joint);
        }

        JsonArray links = json.at("links")->as_array();
        for (auto l : links)
        {
            JsonObject linkObj = l->as_object();

            bool hasName = linkObj.find("name") != linkObj.end();
            bool hasVisual = linkObj.find("visual") != linkObj.end();
            if (!hasName || !hasVisual)
            {
                throw std::runtime_error("Invalid link JSON");
            }
            std::string linkName = linkObj.at("name")->as_string();

            JsonObject visual = linkObj.at("visual")->as_object();

            linalg::Vector o(6, 0);
            bool hasOrigin = visual.find("origin") != linkObj.end();
            if (hasOrigin)
            {
                JsonObject origin = visual.at("origin")->as_object();
                bool hasXYZ = origin.find("xyz") != origin.end();
                bool hasRPY = origin.find("rpy") != origin.end();

                if (!hasXYZ || !hasRPY)
                {
                    throw std::runtime_error("Invalid origin JSON");
                }

                JsonArray xyz = origin["xyz"]->as_array();
                JsonArray rpy = origin["rpy"]->as_array();
                if (xyz.size() != 3 || rpy.size() != 3)
                {
                    throw std::runtime_error("Invalid size for origin array");
                }

                o.at(0) = xyz[0]->as_double();
                o.at(1) = xyz[1]->as_double();
                o.at(2) = xyz[2]->as_double();
                o.at(3) = rpy[0]->as_double();
                o.at(4) = rpy[1]->as_double();
                o.at(5) = rpy[2]->as_double();
            }

            kt.addLink(linkName, Link(o));
        }
        return kt;
    }

    KinematicTree::KinematicTree(const KinematicTree &t)
    {
        this->jointNames = t.jointNames;
        this->linkNames = t.linkNames;
        this->joints = t.joints;
        this->links = t.links;
        this->root = t.root;
        this->endEffector = t.endEffector;
        this->name = t.name;
    }

    KinematicTree &KinematicTree::operator=(const KinematicTree &t)
    {
        if (this == &t)
        {
            return *this;
        }

        this->jointNames = t.jointNames;
        this->linkNames = t.linkNames;
        this->joints = t.joints;
        this->links = t.links;
        this->root = t.root;
        this->endEffector = t.endEffector;
        this->name = t.name;

        return *this;
    }

    std::string KinematicTree::getName() const
    {
        return name;
    }

    Joint KinematicTree::getJoint(const std::string &name) const
    {
        if (joints.find(name) == joints.end())
        {
            throw std::runtime_error("Joint not found");
        }
        return joints.at(name).joint;
    }

    Link KinematicTree::getLink(const std::string &name) const 
    {
        if (links.find(name) == links.end())
        {
            throw std::runtime_error("Link not found");
        }
        return links.at(name).link;
    }

    void KinematicTree::addJoint(const std::string &name, const std::string &parent, const std::string &child, const Joint &joint)
    {
        if (jointNames.find(name) != jointNames.end())
        {
            throw std::runtime_error("Joint already exists");
        }

        jointNames.insert(name);

        JointNode node;
        node.name = name;
        node.parent = parent;
        node.child = child;
        node.joint = joint;

        if (linkNames.find(parent) == linkNames.end())
        {
            linkNames.insert(parent);
            links[parent].name = parent;
        }
        links[parent].children.push_back(name);

        if (linkNames.find(child) == linkNames.end())
        {
            linkNames.insert(child);
            links[child].name = child;
        }
        links[child].parent = name;

        joints[name] = node;
    }

    void KinematicTree::addLink(const std::string &name, const Link &link)
    {
        if (linkNames.find(name) == linkNames.end())
        {
            throw std::runtime_error("Link's relative joint has not been initialized");
        }

        links[name].link = link;
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

    std::vector<std::string> KinematicTree::getJointNames() const
    {
        return std::vector<std::string>(jointNames.begin(), jointNames.end());
    }

    std::vector<std::string> KinematicTree::getLinkNames() const
    {
        return std::vector<std::string>(linkNames.begin(), linkNames.end());
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

    void KinematicTree::setJointState(const std::string &name, double q)
    {
        if (joints.find(name) == joints.end())
        {
            throw std::runtime_error("Joint not found");
        }

        joints.at(name).joint.setState(q);
    }

    void KinematicTree::setJointStates(const sensor_msgs::JointState &q)
    {
        if (q.name.size() != q.position.size())
        {
            throw std::runtime_error("Joint names and positions must have the same size");
        }

        for (int i = 0; i < q.name.size(); i++)
        {
            setJointState(q.name[i], q.position[i]);
        }
    }

    void KinematicTree::setJointStates(const std::vector<double> &q)
    {
        if (q.size() != jointNames.size())
        {
            throw std::runtime_error("Joint positions must have the same size as the number of joints");
        }

        int i = 0;
        for (const auto &name : jointNames)
        {
            setJointState(name, q[i++]);
        }
    }

    void KinematicTree::setJointStates(const std::map<std::string, double> &q)
    {
        for (const auto &j : q)
        {
            setJointState(j.first, j.second);
        }
    }

    double KinematicTree::getJointState(const std::string &name) const
    {
        if (joints.find(name) == joints.end())
        {
            throw std::runtime_error("Joint not found");
        }

        return joints.at(name).joint.getState();
    }

    std::vector<double> KinematicTree::getJointStates() const
    {
        std::vector<double> q;

        for (const auto &j : joints)
        {
            q.push_back(j.second.joint.getState());
        }

        return q;
    }

    std::map<std::string, double> KinematicTree::getJointStatesMap() const
    {
        std::map<std::string, double> q;

        for (const auto &j : joints)
        {
            q[j.first] = j.second.joint.getState();
        }

        return q;
    }

    sensor_msgs::JointState KinematicTree::getJointStateMsg() const
    {
        sensor_msgs::JointState js;
        js.header.frame_id = name;
        js.header.stamp = Time::getTimeNow();

        for (const auto &j : joints)
        {
            js.name.push_back(j.first);
            js.position.push_back(j.second.joint.getState());
            js.velocity.push_back(0);
            js.effort.push_back(0);
        }

        return js;
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

        // return T.applyTransform(getGlobalLinkTransform(parent));
        return getGlobalLinkTransform(parent).applyTransform(T);
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

        // return T.applyTransform(getGlobalJointTransform(parent));
        return getGlobalJointTransform(parent).applyTransform(T);
    }

    std::map<std::string, Transform> KinematicTree::getJointTransforms() const
    {
        std::map<std::string, Transform> transforms;

        for (const auto &j : joints)
        {
            transforms[j.first] = getJointTransform(j.first);
        }

        return transforms;
    }

    std::map<std::string, Transform> KinematicTree::getLinkTransforms() const
    {
        std::map<std::string, Transform> transforms;

        for (const auto &l : links)
        {
            transforms[l.first] = getLinkTransform(l.first);
        }

        return transforms;
    }

    void KinematicTree::getGlobalTransforms(std::map<std::string, Transform> &jointTransforms, std::map<std::string, Transform> &linkTransforms) const
    {
        getGlobalLinkTransform(root, Transform::identity(), jointTransforms, linkTransforms);
    }

    geometry_msgs::TF KinematicTree::TF() const
    {
        std_msgs::Time stamp = Time::getTimeNow();
        geometry_msgs::TF tf;
        int seq = 0;

        for (const auto &l : links)
        {
            geometry_msgs::TransformStamped t;
            t.header.stamp = stamp;

            Transform transform;
            if (l.first == root)
            {
                t.header.frame_id = "world";
                transform = l.second.link.getTransform();
            }
            else
            {
                t.header.frame_id = joints.at(l.second.parent).parent; // Link grandparent
                transform = l.second.link.getTransform(); // T = parent_joint -> current_link
                transform = joints.at(l.second.parent).joint.getTransform().applyTransform(transform); // T = T * (grandparent_link -> parent_joint)
            }
            t.child_frame_id = l.first;
            t.header.seq = seq++;

            t.transform.translation = transform.getTranslation().toMsg();
            t.transform.rotation = transform.getRotation().toMsg();

            tf.transforms.push_back(t);
        }

        return tf;
    }

    geometry_msgs::TF KinematicTree::globalTF() const
    {
        std_msgs::Header header;
        header.frame_id = "world";
        header.seq.data = 0;
        header.stamp = Time::getTimeNow();

        geometry_msgs::TF tf;

        getGlobalLinkTransform(root, Transform::identity(), tf, header);

        return tf;
    }

    linalg::Matrix KinematicTree::getJacobian(const std::string &endEffector) const
    {
        std::string currLink = endEffector;
        linalg::Matrix J(6, 0);

        std::map<std::string, Transform> jointTransforms;
        std::map<std::string, Transform> linkTransforms;
        getGlobalTransforms(jointTransforms, linkTransforms);

        linalg::Vector endEffectorWorld = linkTransforms.at(endEffector).getTranslation();

        while (!currLink.empty() && currLink.compare(root) != 0)
        {
            std::string parentJoint = getParentLink(currLink);
            Transform T = jointTransforms.at(parentJoint);

            linalg::Vector jointOriginWorld = T.getTranslation();

            linalg::Vector jointAxis = joints.at(parentJoint).joint.getAxis();
            linalg::Vector jointAxisHomogeneous = jointAxis;
            jointAxisHomogeneous.push_back(0);

            linalg::Vector jointAxisWorld = linalg::Matrix::multiply(T.getMatrix(), jointAxisHomogeneous).subvec(0, 3);
            linalg::Vector vecToEndeffector = endEffectorWorld - jointOriginWorld;

            linalg::Vector linearVelocity = linalg::Vector::cross(jointAxisWorld, vecToEndeffector);

            if (joints.at(parentJoint).joint.getType() == JointType::REVOLUTE)
            {
                linalg::Vector jointVelocity = linearVelocity;
                jointVelocity.push_back(jointAxisWorld.get(0));
                jointVelocity.push_back(jointAxisWorld.get(1));
                jointVelocity.push_back(jointAxisWorld.get(2));
                J.pushCol(jointVelocity);
            }
            else if (joints.at(parentJoint).joint.getType() == JointType::PRISMATIC)
            {
                linalg::Vector jointVelocity = jointAxisWorld;
                jointVelocity.push_back(0);
                jointVelocity.push_back(0);
                jointVelocity.push_back(0);
                J.pushCol(jointVelocity);
            }

            currLink = getParentJoint(parentJoint);
        }
        return J;
    }

    bool KinematicTree::solveIK(sensor_msgs::JointState &js, const Transform &endEffectorGoal, double tol, int &maxIter)
    {

    }

    void KinematicTree::getGlobalJointTransform(const std::string &name, const Transform &transform, geometry_msgs::TF &tf, std_msgs::Header header) const
    {
        if (joints.find(name) == joints.end())
        {
            throw std::runtime_error("Joint not found");
        }

        JointNode node = joints.at(name);
        Transform T = node.joint.getTransform(); // T = parent_link -> current_joint
        T = transform.applyTransform(T); // T = (world -> parent_link) * T

        getGlobalLinkTransform(node.child, T, tf, header);
    }

    void KinematicTree::getGlobalLinkTransform(const std::string &name, const Transform &transform, geometry_msgs::TF &tf, std_msgs::Header header) const
    {
        if (links.find(name) == links.end())
        {
            throw std::runtime_error("Link not found");
        }

        LinkNode node = links.at(name);
        header.seq.data += 1;

        Transform T;
        T = node.link.getTransform(); // T = parent_joint -> current_link
        T = transform.applyTransform(T); // T = (world -> parent_joint) * T

        geometry_msgs::TransformStamped t;
        t.transform.translation = T.getTranslation().toMsg();
        t.transform.rotation = T.getRotation().toMsg();
        t.header = header;
        t.child_frame_id = name;

        tf.transforms.push_back(t);

        for (const auto &child : node.children)
        {
            getGlobalJointTransform(child, T, tf, header);
        }
    }

    void KinematicTree::getGlobalJointTransform(const std::string &name, const Transform &transform, std::map<std::string, Transform> &jointTransforms, std::map<std::string, Transform> &linkTransforms) const
    {
        if (joints.find(name) == joints.end())
        {
            throw std::runtime_error("Joint not found");
        }

        JointNode node = joints.at(name);
        Transform T = node.joint.getTransform();
        T = T.applyTransform(transform);

        jointTransforms[name] = T;

        getGlobalLinkTransform(node.child, T, jointTransforms, linkTransforms);
    }

    void KinematicTree::getGlobalLinkTransform(const std::string &name, const Transform &transform, std::map<std::string, Transform> &jointTransforms, std::map<std::string, Transform> &linkTransforms) const
    {
        if (links.find(name) == links.end())
        {
            throw std::runtime_error("Link not found");
        }

        LinkNode node = links.at(name);
        Transform T = node.link.getTransform();
        T = T.applyTransform(transform);

        linkTransforms[name] = T;

        for (const auto &child : node.children)
        {
            getGlobalJointTransform(child, T, jointTransforms, linkTransforms);
        }
    }

} // namespace kineval