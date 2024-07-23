cmake_minimum_required(VERSION 3.5)

project(mros)

set(CMAKE_CXX_STANDARD 11)

find_library(PTHREAD pthread)

### UTILS LIBRARY ###

add_library(utils STATIC src/utils.cpp)
target_include_directories(utils PRIVATE include)

### MESSAGES LIBRARY ###

add_library(msgs STATIC src/messages/message.cpp
                            src/messages/geometry_msgs/point.cpp
                            src/messages/geometry_msgs/pose.cpp
                            src/messages/geometry_msgs/quaternion.cpp
                            src/messages/geometry_msgs/transform.cpp
                            src/messages/geometry_msgs/twist.cpp
                            src/messages/geometry_msgs/vector3.cpp
                            src/messages/mbot_msgs/encoders.cpp
                            src/messages/mbot_msgs/mbotImu.cpp
                            src/messages/mbot_msgs/motorPwm.cpp
                            src/messages/mbot_msgs/motorVel.cpp
                            src/messages/mbot_msgs/pose2D.cpp
                            src/messages/mbot_msgs/timestamp.cpp
                            src/messages/mbot_msgs/twist2D.cpp
                            src/messages/sensor_msgs/jointState.cpp
                            src/messages/std_msgs/string.cpp
                            src/messages/std_msgs/time.cpp
                            src/messages/std_msgs/header.cpp
                            src/messages/std_msgs/integer.cpp
                            src/messages/std_msgs/float.cpp
                            src/messages/std_msgs/color.cpp)
target_include_directories(msgs PRIVATE include)
target_link_libraries(msgs utils)

### SHARED MEMORY LIBRARY ###

add_library(smem STATIC src/smem/squeue.cpp
                           src/smem/smutex.cpp
                           src/smem/smem.cpp)
target_include_directories(smem PRIVATE include)
target_link_libraries(smem pthread utils)

### SOCKET LIBRARY ###

add_library(socket STATIC src/socket/tcp/client.cpp
                             src/socket/tcp/server.cpp
                             src/socket/udp/client.cpp
                             src/socket/udp/server.cpp)
target_include_directories(socket PRIVATE include)
target_link_libraries(socket utils)

### LINALG LIBRARY ###

add_library(linalg STATIC src/linalg/vector.cpp
                          src/linalg/matrix.cpp
                          src/linalg/quaternion.cpp
                          src/linalg/rotation.cpp)
target_include_directories(linalg PRIVATE include)
target_link_libraries(linalg msgs)

### KINEVAL LIBRARY ###
add_library(kineval STATIC src/kineval/joint.cpp
                           src/kineval/json.cpp
                           src/kineval/link.cpp
                           src/kineval/transform.cpp
                           src/kineval/tree.cpp)
target_include_directories(kineval PRIVATE include)
target_link_libraries(kineval linalg)

### MROS PRIVATE MESSAGES LIBRARY ###
add_library(mros_priv_msgs STATIC src/mros/private_msgs/disconnect.cpp
                                      src/mros/private_msgs/notify.cpp
                                      src/mros/private_msgs/register.cpp
                                      src/mros/private_msgs/request.cpp
                                      src/mros/private_msgs/response.cpp
                                      src/mros/private_msgs/uri.cpp)
target_include_directories(mros_priv_msgs PRIVATE include)
target_link_libraries(mros_priv_msgs msgs)

### MROS LIBRARY ###

add_library(mros STATIC src/mros/mediator.cpp
                        src/mros/node.cpp
                        src/mros/publisher.cpp
                        src/mros/subscriber.cpp
                        src/mros/signalHandler.cpp
                        src/mros/console.cpp
                        src/mros/optparse.cpp)
target_include_directories(mros PRIVATE include)
target_link_libraries(mros socket smem mros_priv_msgs)

### MROS TESTS ###

# Add core test
add_executable(core_test test/mros/coreTest.cpp)
target_link_libraries(core_test mros)
target_include_directories(core_test PRIVATE include)

# Add publisher test
add_executable(publisher_test test/mros/publisherTest.cpp)
target_link_libraries(publisher_test mros)
target_include_directories(publisher_test PRIVATE include)

# Add subscriber test
add_executable(subscriber_test test/mros/subscriberTest.cpp)
target_link_libraries(subscriber_test mros)
target_include_directories(subscriber_test PRIVATE include)

# Add pubsub test
add_executable(pubsub_test test/mros/pubsubTest.cpp)
target_link_libraries(pubsub_test mros)
target_include_directories(pubsub_test PRIVATE include)

# Add pose publisher test
add_executable(pose_publisher_test test/mros/demo1/posePublisherTest.cpp)
target_link_libraries(pose_publisher_test mros linalg)
target_include_directories(pose_publisher_test PRIVATE include)

# Add twist publisher test
add_executable(twist_publisher_test test/mros/demo1/twistPublisherTest.cpp)
target_link_libraries(twist_publisher_test mros linalg)
target_include_directories(twist_publisher_test PRIVATE include)

# Add pose listener test
add_executable(pose_listener_test test/mros/demo1/poseListenerTest.cpp)
target_link_libraries(pose_listener_test mros linalg)
target_include_directories(pose_listener_test PRIVATE include)

# Add twist listener test
add_executable(twist_listener_test test/mros/demo1/twistListenerTest.cpp)
target_link_libraries(twist_listener_test mros linalg)
target_include_directories(twist_listener_test PRIVATE include)

# Add console test
add_executable(console_test test/mros/consoleTest.cpp)
target_link_libraries(console_test mros)
target_include_directories(console_test PRIVATE include)

# Add argparse test
add_executable(optparse_test test/mros/optparseTest.cpp)
target_link_libraries(optparse_test mros)
target_include_directories(optparse_test PRIVATE include)

# Add joint state publisher test
add_executable(joint_state_publisher test/mros/demo2/jointStatePublisher.cpp)
target_link_libraries(joint_state_publisher mros)
target_include_directories(joint_state_publisher PRIVATE include)

# Add robot state publisher test
add_executable(robot_state_publisher test/mros/demo2/robotStatePublisher.cpp)
target_link_libraries(robot_state_publisher mros kineval)
target_include_directories(robot_state_publisher PRIVATE include)

# Add robot state listener test
add_executable(robot_state_listener test/mros/demo2/robotStateListener.cpp)
target_link_libraries(robot_state_listener mros kineval)
target_include_directories(robot_state_listener PRIVATE include)

# Add joint group 1 publisher test
add_executable(joint_group_1 test/mros/demo2/jointGroup1.cpp)
target_link_libraries(joint_group_1 mros kineval)
target_include_directories(joint_group_1 PRIVATE include)

# Add joint group 2 publisher test
add_executable(joint_group_2 test/mros/demo2/jointGroup2.cpp)
target_link_libraries(joint_group_2 mros kineval)
target_include_directories(joint_group_2 PRIVATE include)

### SOCKET TESTS ###

# Add socket client test
add_executable(tcp_client_test test/socket/tcp/clientTest.cpp)
target_link_libraries(tcp_client_test socket)
target_include_directories(tcp_client_test PRIVATE include)

# Add socket server test
add_executable(tcp_server_test test/socket/tcp/serverTest.cpp)
target_link_libraries(tcp_server_test socket)
target_include_directories(tcp_server_test PRIVATE include)

# Add socket client test
add_executable(tcp_client_noblock_test test/socket/tcp/clientNoBlockTest.cpp)
target_link_libraries(tcp_client_noblock_test socket)
target_include_directories(tcp_client_noblock_test PRIVATE include)

# Add socket server test
add_executable(tcp_server_noblock_test test/socket/tcp/serverNoBlockTest.cpp)
target_link_libraries(tcp_server_noblock_test socket)
target_include_directories(tcp_server_noblock_test PRIVATE include)

# Add UDP socket client test
add_executable(udp_client_test test/socket/udp/clientTest.cpp)
target_link_libraries(udp_client_test socket)
target_include_directories(udp_client_test PRIVATE include)

# Add UDP socket server test
add_executable(udp_server_test test/socket/udp/serverTest.cpp)
target_link_libraries(udp_server_test socket)
target_include_directories(udp_server_test PRIVATE include)

# Add UDP socket peer test
add_executable(udp_peer_test test/socket/udp/peerTest.cpp)
target_link_libraries(udp_peer_test socket)
target_include_directories(udp_peer_test PRIVATE include)

### SHARED MEMORY TESTS ###

# Add smem read test
add_executable(smem_read_test test/smem/smemReadTest.cpp)
target_link_libraries(smem_read_test smem)
target_include_directories(smem_read_test PUBLIC include)

# Add smem write test
add_executable(smem_write_test test/smem/smemWriteTest.cpp)
target_link_libraries(smem_write_test smem)
target_include_directories(smem_write_test PUBLIC include)

# Add smem mutex test
add_executable(smem_mutex_test test/smem/smemMutexTest.cpp)
target_link_libraries(smem_mutex_test smem)
target_include_directories(smem_mutex_test PUBLIC include)

### MESSAGES TESTS ###

# Add pose messages test
add_executable(pose_test test/messages/poseTest.cpp)
target_link_libraries(pose_test msgs)
target_include_directories(pose_test PUBLIC include)

# Add jointState messages test
add_executable(joint_state_test test/messages/jointStateTest.cpp)
target_link_libraries(joint_state_test msgs)
target_include_directories(joint_state_test PUBLIC include)

### LINALG TESTS ###

# Add linalg vector test
add_executable(vector_test test/linalg/vectorTest.cpp)
target_link_libraries(vector_test linalg)
target_include_directories(vector_test PUBLIC include)

# Add linalg matrix test
add_executable(matrix_test test/linalg/matrixTest.cpp)
target_link_libraries(matrix_test linalg)
target_include_directories(matrix_test PUBLIC include)

# Add linalg quaternion test
add_executable(quaternion_test test/linalg/quaternionTest.cpp)
target_link_libraries(quaternion_test linalg)
target_include_directories(quaternion_test PUBLIC include)

### KINEVAL TESTS ###

# Add kineval test
add_executable(tree_test test/kineval/treeTest.cpp)
target_link_libraries(tree_test kineval linalg)
target_include_directories(tree_test PUBLIC include)


### INSTALL ###

# Install libraries
install(TARGETS utils msgs smem socket linalg kineval mros_priv_msgs mros
        EXPORT mrosTargets
        ARCHIVE DESTINATION /usr/local/lib/mros)

# Install header files while preserving directory structure
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ 
        DESTINATION /usr/local/include/mros
        FILES_MATCHING PATTERN "*.hpp")

install(FILES mrosConfig.cmake DESTINATION /usr/local/lib/cmake/mros)

install(EXPORT mrosTargets
        FILE mrosTargets.cmake
        DESTINATION /usr/local/lib/cmake/mros)