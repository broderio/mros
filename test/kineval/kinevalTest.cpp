#include "kineval/tree.hpp"

int main() {
    std::ifstream file("/Users/broderio/Repositories/simple_pubsub/robots/simpleBot.json");

    auto json = kineval::JsonParser::parse(file);

    std::cout << kineval::JsonParser::stringify(json) << std::endl;

    kineval::KinematicTree kt = kineval::KinematicTree::fromJson(json->as_object());

    std::cout << "Name: " << kt.getName() << std::endl;
    std::cout << "Joint Count: " << kt.getJointNames().size() << std::endl;
    std::cout << "Link Count: " << kt.getLinkNames().size() << std::endl;

    kineval::Transform T = kt.getGlobalLinkTransform(kt.getEndEffector());

    linalg::Matrix t = T.getTransform();
    std::cout << "End Effector Transform:\n" << t << std::endl;


    kt.setJointState("elbow", M_PI / 2);
    T = kt.getGlobalLinkTransform(kt.getEndEffector());
    t = T.getTransform();
    std::cout << "End Effector Transform:\n" << t << std::endl;
}