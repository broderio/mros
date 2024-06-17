#include <iostream>
#include <vector>
#include <string>

#include "utils.hpp"

#include "socket/tcp/server.hpp"
#include "socket/tcp/client.hpp"

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "linalg/quaternion.hpp"

#include "messages/sensor_msgs/jointState.hpp"

#include "messages/geometry_msgs/transform.hpp"
#include "messages/geometry_msgs/vector3.hpp"
#include "messages/geometry_msgs/quaternion.hpp"
#include "messages/geometry_msgs/pose.hpp"
#include "messages/geometry_msgs/point.hpp"


int main() {
    std::string ipAddr = getIPAddr();
    TCPClient client(ipAddr, 9002, false);
    int res = client.connect();
    if (res < 0) {
        return 1;
    }

    // Create identity matrix for global frame
    linalg::Matrix H0 = linalg::Matrix::identity(4);

    // Create translation and axis for joint 1
    geometry_msgs::Vector3 t1(1.0, 0.0, 0.0);
    geometry_msgs::Vector3 axis1(0.0, 1.0, 0.0);

    // Create translation and axis for joint 2
    geometry_msgs::Vector3 t2(1.0, 0.0, 0.0);
    geometry_msgs::Vector3 axis2(0.0, 1.0, 0.0);

    // Create translation for endeffector
    geometry_msgs::Vector3 t3(1.0, 0.0, 0.0);

    Header header;
    std::vector<geometry_msgs::TransformStamped> tfList(2);
    tfList[0] = geometry_msgs::TransformStamped(header, geometry_msgs::Transform(t1, linalg::Quaternion(axis1, 0.0).toMsg()));
    tfList[1] = geometry_msgs::TransformStamped(header, geometry_msgs::Transform(t2, linalg::Quaternion(axis2, 0.0).toMsg()));

    for (int i = 0; i < 10; ++i) {
        // Get joint state message
        sensor_msgs::JointState js;
        std::string msg;
        client.receive(msg, 400);
        Parser::decode(msg, js);

        t1 = tfList[0].transform.translation;
        t2 = tfList[1].transform.translation;

        // Update translation and rotation for joint 1
        float angle1 = js.position.at(0);
        linalg::Quaternion q1(axis1, angle1);
        linalg::Matrix R1 = linalg::Quaternion::toRotationMatrix(q1);
        linalg::Matrix T1 = linalg::Matrix::translation(t1.x, t1.y, t1.z);
        linalg::Matrix H1 = linalg::Matrix::multiply(T1, R1);

        // Update translation and rotation for joint 2
        float angle2 = js.position.at(1);
        linalg::Quaternion q2(axis2, angle2);
        linalg::Matrix R2 = linalg::Quaternion::toRotationMatrix(q2);
        linalg::Matrix T2 = linalg::Matrix::translation(t2.x, t2.y, t2.z);
        linalg::Matrix H2 = linalg::Matrix::multiply(H1, linalg::Matrix::multiply(T2, R2));

        // Create header
        header.seq = i;
        header.stamp.sec = getTimeNano() / 1000000000;
        header.stamp.nsec = getTimeNano() % 1000000000;
        header.frame_id = String("world");

        // Create transform list
        header.frame_id = String("joint1");
        tfList[0] = geometry_msgs::TransformStamped(header, geometry_msgs::Transform(t1, q1.toMsg()));

        header.frame_id = String("joint2");
        tfList[1] = geometry_msgs::TransformStamped(header, geometry_msgs::Transform(t2, q2.toMsg()));

        // Update joint pose
        std::vector<geometry_msgs::Pose> jointPoses(3);
        linalg::Vector tVec(std::vector<float>({t1.x, t1.y, t1.z, 1.0}));
        jointPoses[0] = geometry_msgs::Pose(geometry_msgs::Point(linalg::Matrix::multiply(H0, tVec).toVector3()), q1.toMsg());
        tVec = linalg::Vector(std::vector<float>({t2.x, t2.y, t2.z, 1.0}));
        jointPoses[1] = geometry_msgs::Pose(geometry_msgs::Point(linalg::Matrix::multiply(H1, tVec).toVector3()), q2.toMsg());
        tVec = linalg::Vector(std::vector<float>({t3.x, t3.y, t3.z, 1.0}));
        jointPoses[2] = geometry_msgs::Pose(geometry_msgs::Point(linalg::Matrix::multiply(H2, tVec).toVector3()), linalg::Quaternion().toMsg());

        // Create tf message
        geometry_msgs::TF tf(tfList);

        // Print tf message
        std::cout << std::endl;
        for (int i = 0; i < tf.transforms.size(); ++i) {
            std::cout << "Joint " << i << ":" << std::endl;
            std::cout << "\tTranslation: " << tf.transforms[i].transform.translation.x << " " << tf.transforms[i].transform.translation.y << " " << tf.transforms[i].transform.translation.z << std::endl;
            std::cout << "\tRotation: " << tf.transforms[i].transform.rotation.x << " " << tf.transforms[i].transform.rotation.y << " " << tf.transforms[i].transform.rotation.z << " " << tf.transforms[i].transform.rotation.w << std::endl;
            std::cout << "\tPose: " << std::endl;
            std::cout << "\t\tPosition: " << jointPoses[i].position.x << " " << jointPoses[i].position.y << " " << jointPoses[i].position.z << std::endl;
            std::cout << "\t\tOrientation: " << jointPoses[i].orientation.x << " " << jointPoses[i].orientation.y << " " << jointPoses[i].orientation.z << " " << jointPoses[i].orientation.w << std::endl;
        }
        std::cout << "Endeffector:" << std::endl;
        std::cout << "Pose: " << std::endl;
        std::cout << "\tPosition: " << jointPoses[2].position.x << " " << jointPoses[2].position.y << " " << jointPoses[2].position.z << std::endl;
        std::cout << "\tOrientation: " << jointPoses[2].orientation.x << " " << jointPoses[2].orientation.y << " " << jointPoses[2].orientation.z << " " << jointPoses[2].orientation.w << std::endl;
    }
    
    return 0;
}