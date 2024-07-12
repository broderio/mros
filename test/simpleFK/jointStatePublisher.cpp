#include <iostream>
#include <vector>
#include <string>

#include "socket/tcp/server.hpp"

#include "linalg/vector.hpp"
#include "linalg/matrix.hpp"
#include "messages/sensor_msgs/jointState.hpp"


int main() {
    TCPServer server(9002, false);
    TCPConnection conn;
    int res = server.accept(conn);
    if (res < 0) {
        return 1;
    }

    // Create joint state message
    sensor_msgs::JointState js;
    js.header.seq = 0;
    js.header.stamp.sec = getTimeNano() / 1000000000;
    js.header.stamp.nsec = getTimeNano() % 1000000000;
    js.header.frame_id = "js_publisher";

    js.name.push_back(String("joint1"));
    js.name.push_back(String("joint2"));

    js.position.push_back(0.0);
    js.position.push_back(0.0);

    js.velocity.push_back(0.0);
    js.velocity.push_back(0.0);

    js.effort.push_back(0.0);
    js.effort.push_back(0.0);

    // Send joint state message
    for (int i = 0; i < 10; i++) {
        std::string msg = Parser::encode(js, 400);
        res = conn.send(msg);
        if (res < 0) {
            return 1;
        }

        js.header.seq.data++;
        js.header.stamp.sec = getTimeNano() / 1000000000;
        js.header.stamp.nsec = getTimeNano() % 1000000000;

        js.position[0].data += 0.1;
        js.position[1].data += 0.05;

        std::cout << "Sent joint state message:" << std::endl;
        std::cout << "Size: " << msg.size() << std::endl;
        std::cout << "Rotation: " << js.position.at(0) << " " << js.position.at(1) << std::endl;

        sleep(250);
    }

}