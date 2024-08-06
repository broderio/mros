#include "kineval/tree.hpp"

#include "messages/message.hpp"
#include "messages/geometry_msgs/transform.hpp"
#include "messages/sensor_msgs/jointState.hpp"

int main() {
    std::ifstream file("/Users/broderio/Repositories/simple_pubsub/robots/mr2.json");

    auto json = kineval::JsonParser::parse(file);

    // std::cout << kineval::JsonParser::stringify(json) << std::endl;

    kineval::KinematicTree kt = kineval::KinematicTree::fromJson(json->as_object());

    std::cout << "\nName: " << kt.getName() << std::endl;
    std::cout << "Joint Count: " << kt.getJointNames().size() << std::endl;
    std::cout << "Link Count: " << kt.getLinkNames().size() << std::endl;

    kineval::Transform T = kt.getGlobalLinkTransform(kt.getEndEffector());

    linalg::Matrix t = T.getMatrix();
    std::cout << "\nEnd Effector Transform:\n" << t << std::endl;
    
    std::cout << "\nTF: " << kt.TF() << std::endl;
    std::cout << "\nGlobal TF:\n" << kt.globalTF() << std::endl;

    kt.setJointState("clavicle_right_yaw", -M_PI / 2);
    T = kt.getGlobalLinkTransform(kt.getEndEffector());
    t = T.getMatrix();
    std::cout << "\nEnd Effector Transform (clavicle_right_yaw.q == 90 deg):\n" << t << std::endl;

    std::cout << "\nGlobal TF (elbow state == 45):\n" << kt.globalTF() << std::endl;
}