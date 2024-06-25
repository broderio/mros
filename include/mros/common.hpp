#ifndef MROS_COMMON_HPP
#define MROS_COMMON_HPP

enum CORE_TOPICS {
    REGISTER, // From nodes to mediator, msgType: private_msgs::Register
    DEREGISTER, // From nodes to mediator, msgType: private_msgs::Register
    REQUEST, // From node to node, msgType: private_msgs::Request
    RESPONSE, // From mediator to node, msgType: private_msgs::URIArray
    CONNECT, // From publisher node to subscriber node, msgType: private_msgs::URIStamped
    DISCONNECT // From mediator to node, msgType: private_msgs::Disconnect
};

#endif // MROS_COMMON_HPP