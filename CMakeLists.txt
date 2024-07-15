cmake_minimum_required(VERSION 3.0)

project(mros)

set(CMAKE_CXX_STANDARD 11)

find_library(PTHREAD pthread)

### UTILS ###

add_library(utils STATIC src/utils.cpp)
target_include_directories(utils PRIVATE include)

### MESSAGES ###

add_library(messagelib STATIC src/messages/message.cpp
                            src/messages/geometry_msgs/point.cpp
                            src/messages/geometry_msgs/pose.cpp
                            src/messages/geometry_msgs/quaternion.cpp
                            src/messages/geometry_msgs/transform.cpp
                            src/messages/geometry_msgs/twist.cpp
                            src/messages/geometry_msgs/vector3.cpp
                            src/messages/sensor_msgs/jointstate.cpp
                            src/messages/std_msgs/string.cpp
                            src/messages/std_msgs/time.cpp
                            src/messages/std_msgs/header.cpp
                            src/messages/std_msgs/integer.cpp
                            src/messages/std_msgs/float.cpp
                            src/messages/std_msgs/color.cpp)
target_include_directories(messagelib PRIVATE include)
target_link_libraries(messagelib utils)

### SHARED MEMORY ###

add_library(smemlib STATIC src/smem/squeue.cpp
                           src/smem/smutex.cpp
                           src/smem/smem.cpp)
target_include_directories(smemlib PRIVATE include)
target_link_libraries(smemlib pthread utils)

### SOCKET ###

add_library(socketlib STATIC src/socket/tcp/client.cpp
                             src/socket/tcp/server.cpp
                             src/socket/udp/client.cpp
                             src/socket/udp/server.cpp)
target_include_directories(socketlib PRIVATE include)
target_link_libraries(socketlib utils)

### LINALG ###

add_library(linalg STATIC src/linalg/vector.cpp
                          src/linalg/matrix.cpp
                          src/linalg/quaternion.cpp)
target_include_directories(linalg PRIVATE include)
target_link_libraries(linalg utils messagelib)

### JRDF ###
add_library(jrdf STATIC src/jrdf/json.cpp
                        src/jrdf/jrdf.cpp)
target_include_directories(jrdf PRIVATE include)
target_link_libraries(jrdf utils)

### MROS PRIVATE MESSAGES ###
add_library(mros_priv_messages STATIC src/mros/private_msgs/disconnect.cpp
                                      src/mros/private_msgs/notify.cpp
                                      src/mros/private_msgs/register.cpp
                                      src/mros/private_msgs/request.cpp
                                      src/mros/private_msgs/response.cpp
                                      src/mros/private_msgs/uri.cpp)
target_include_directories(mros_priv_messages PRIVATE include)
target_link_libraries(mros_priv_messages messagelib)

### MROS ###

add_library(mros STATIC src/mros/mediator.cpp
                        src/mros/node.cpp
                        src/mros/publisher.cpp
                        src/mros/subscriber.cpp
                        src/mros/signalHandler.cpp
                        src/mros/console.cpp)
target_include_directories(mros PRIVATE include)
target_link_libraries(mros socketlib smemlib utils messagelib mros_priv_messages)

### MROS TESTS ###

# Add core test
add_executable(core_test test/mros/core_test.cpp)
target_link_libraries(core_test mros)
target_include_directories(core_test PRIVATE include)

# Add publisher test
add_executable(publisher_test test/mros/publisher_test.cpp)
target_link_libraries(publisher_test mros)
target_include_directories(publisher_test PRIVATE include)

# Add subscriber test
add_executable(subscriber_test test/mros/subscriber_test.cpp)
target_link_libraries(subscriber_test mros)
target_include_directories(subscriber_test PRIVATE include)

# Add pubsub test
add_executable(pubsub_test test/mros/pubsub_test.cpp)
target_link_libraries(pubsub_test mros)
target_include_directories(pubsub_test PRIVATE include)

# Add pose publisher test
add_executable(pose_publisher_test test/mros/demo1/pose_publisher_test.cpp)
target_link_libraries(pose_publisher_test mros linalg)
target_include_directories(pose_publisher_test PRIVATE include)

# Add twist publisher test
add_executable(twist_publisher_test test/mros/demo1/twist_publisher_test.cpp)
target_link_libraries(twist_publisher_test mros linalg)
target_include_directories(twist_publisher_test PRIVATE include)

# Add pose listener test
add_executable(pose_listener_test test/mros/demo1/pose_listener_test.cpp)
target_link_libraries(pose_listener_test mros linalg)
target_include_directories(pose_listener_test PRIVATE include)

# Add twist listener test
add_executable(twist_listener_test test/mros/demo1/twist_listener_test.cpp)
target_link_libraries(twist_listener_test mros linalg)
target_include_directories(twist_listener_test PRIVATE include)

# Add console test
add_executable(console_test test/mros/console_test.cpp)
target_link_libraries(console_test mros)
target_include_directories(console_test PRIVATE include)

### SOCKET TESTS ###

# Add socket client test
add_executable(tcp_client_test test/socket/tcp/client_test.cpp)
target_link_libraries(tcp_client_test socketlib)
target_include_directories(tcp_client_test PRIVATE include)

# Add socket server test
add_executable(tcp_server_test test/socket/tcp/server_test.cpp)
target_link_libraries(tcp_server_test socketlib)
target_include_directories(tcp_server_test PRIVATE include)

# Add socket client test
add_executable(tcp_client_noblock_test test/socket/tcp/client_noblock_test.cpp)
target_link_libraries(tcp_client_noblock_test socketlib)
target_include_directories(tcp_client_noblock_test PRIVATE include)

# Add socket server test
add_executable(tcp_server_noblock_test test/socket/tcp/server_noblock_test.cpp)
target_link_libraries(tcp_server_noblock_test socketlib)
target_include_directories(tcp_server_noblock_test PRIVATE include)

# Add UDP socket client test
add_executable(udp_client_test test/socket/udp/client_test.cpp)
target_link_libraries(udp_client_test socketlib)
target_include_directories(udp_client_test PRIVATE include)

# Add UDP socket server test
add_executable(udp_server_test test/socket/udp/server_test.cpp)
target_link_libraries(udp_server_test socketlib)
target_include_directories(udp_server_test PRIVATE include)

#Add UDP socket peer test
add_executable(udp_peer_test test/socket/udp/peer_test.cpp)
target_link_libraries(udp_peer_test socketlib)
target_include_directories(udp_peer_test PRIVATE include)

### SHARED MEMORY TESTS ###

# Add smem read test
add_executable(smemReadTest test/smem/smemReadTest.cpp)
target_link_libraries(smemReadTest smemlib)
target_include_directories(smemReadTest PUBLIC include)

# Add smem write test
add_executable(smemWriteTest test/smem/smemWriteTest.cpp)
target_link_libraries(smemWriteTest smemlib)
target_include_directories(smemWriteTest PUBLIC include)

# Add smem mutex test
add_executable(smemMutexTest test/smem/smemMutexTest.cpp)
target_link_libraries(smemMutexTest smemlib)
target_include_directories(smemMutexTest PUBLIC include)

### MESSAGES TESTS ###

# Add pose messages test
add_executable(poseTest test/messages/poseTest.cpp)
target_link_libraries(poseTest messagelib)
target_include_directories(poseTest PUBLIC include)

# Add jointState messages test
add_executable(jointStateTest test/messages/jointStateTest.cpp)
target_link_libraries(jointStateTest messagelib)
target_include_directories(jointStateTest PUBLIC include)

### LINALG TESTS ###

# Add linalg vector test
add_executable(vectorTest test/linalg/vectorTest.cpp)
target_link_libraries(vectorTest linalg)
target_include_directories(vectorTest PUBLIC include)

# Add linalg matrix test
add_executable(matrixTest test/linalg/matrixTest.cpp)
target_link_libraries(matrixTest linalg)
target_include_directories(matrixTest PUBLIC include)

# Add linalg quaternion test
add_executable(quaternionTest test/linalg/quaternionTest.cpp)
target_link_libraries(quaternionTest linalg)
target_include_directories(quaternionTest PUBLIC include)

### JRDF TESTS ###
add_executable(jsonTest test/jrdf/jsonTest.cpp)
target_link_libraries(jsonTest jrdf)
target_include_directories(jsonTest PRIVATE include)

### COMBINED TESTS ###

add_executable(jointStatePublisher test/simpleFK/jointStatePublisher.cpp)
target_link_libraries(jointStatePublisher socketlib linalg utils messagelib)
target_include_directories(jointStatePublisher PRIVATE include)

add_executable(jointStateSubscription test/simpleFK/jointStateSubscription.cpp)
target_link_libraries(jointStateSubscription socketlib linalg utils messagelib)
target_include_directories(jointStateSubscription PRIVATE include)