#ifndef MROS_CORE_HPP
#define MROS_CORE_HPP

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <thread>
#include <chrono>

#include "utils.hpp"

#include "socket/tcp/server.hpp"
#include "socket/udp/server.hpp"

#include "messages/message.hpp"
#include "messages/std_msgs/header.hpp"
#include "messages/std_msgs/time.hpp"
#include "messages/std_msgs/string.hpp"

#include "mros/private_msgs/register.hpp"
#include "mros/private_msgs/request.hpp"
#include "mros/private_msgs/disconnect.hpp"
#include "mros/private_msgs/uri.hpp"

#include "mros/common.hpp"
#include "mros/publisher.hpp"
#include "mros/subscriber.hpp"

namespace mros {

class Node {
public:
    static void spin();
    static void spinOnce();

    template<typename T>
    static Publisher<T> advertise(const std::string& topic, const size_t& queueSize) {
        return Publisher<T>(topic, queueSize);
    }

    template<typename T>
    static Subscriber<T> subscribe(std::function<void(const T&)> callback, const std::string& topic, const size_t& queueSize) {
        return Subscriber<T>(callback, topic, queueSize);
    }

private:
    static void run();

    static std::map<std::string, std::shared_ptr<IPublisher>> publishers; // Map of topic names to Publisher objects
    static std::map<std::string, std::shared_ptr<ISubscriber>> subscribers; // Map of topic names to Subscriber objects
};

}

#endif // MROS_CORE_HPP