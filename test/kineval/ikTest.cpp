#include "kineval/tree.hpp"

#include "messages/message.hpp"
#include "messages/geometry_msgs/transform.hpp"
#include "messages/sensor_msgs/jointState.hpp"

int main() {
    std::ifstream file("/Users/broderio/Repositories/simple_pubsub/robots/mr2.json");

    auto json = kineval::JsonParser::parse(file);

    kineval::KinematicTree kt = kineval::KinematicTree::fromJson(json->as_object());

    while (true)
    {

        for (auto &name : kt.getJointNames())
        {
            if (kt.getJoint(name).getType() == kineval::JointType::FIXED)
            {
                continue;
            }

            double l, u;
            kt.getJoint(name).getLimits(l, u);
            std::cout << "Input state for \"" << name << "\"" << " [" << l << ", " << u << "]" << std::endl;
            double q;
            std::cin >> q;
            kt.setJointState(name, q);
        }

        sensor_msgs::JointState js;
        kineval::Transform goal;
        goal.setTranslation(kt.getGlobalLinkTransform(kt.getEndEffector()).getTranslation());

        std::cout << "Jacobian:\n" << kt.getJacobian(kt.getEndEffector()) << std::endl;

        // for (auto &name : kt.getJointNames())
        // {
        //     kt.setJointState(name, 0);
        // }

        // int maxIter = 10000;
        // if (kt.solveIK(js, goal, 0.005, maxIter))
        // {
        //     std::cout << "Number of iterations: " << maxIter << std::endl;
        //     std::cout << "End Effector Goal:\n" << goal.getMatrix() << std::endl;
        //     std::cout << "\nJoint States: \n" << js << std::endl;
        //     kt.setJointStates(js);
        //     std::cout << "\nEnd Effector Position: " << kt.getGlobalLinkTransform(kt.getEndEffector()).getTranslation() << std::endl;
        // }
        // else
        // {
        //     std::cout << "IK unsuccessful" << std::endl;
        // }
    }
}