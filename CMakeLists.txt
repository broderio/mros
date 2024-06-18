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
                            src/messages/mbot_msgs/lidarScan.cpp
                            src/messages/mbot_msgs/mbotEncoders.cpp
                            src/messages/mbot_msgs/pose2d.cpp
                            src/messages/mbot_msgs/twist2d.cpp)
target_include_directories(messagelib PRIVATE include)

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

### MROS ###

add_library(mros STATIC src/mros/core.cpp
                        src/mros/publisher.cpp
                        src/mros/subscriber.cpp)
target_include_directories(mros PRIVATE include)
target_link_libraries(mros socketlib smemlib utils)

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

# Add pose2d messages test
add_executable(pose2dTest test/messages/pose2dTest.cpp)
target_link_libraries(pose2dTest messagelib)
target_include_directories(pose2dTest PUBLIC include)

# Add twist2d messages test
add_executable(twist2dTest test/messages/twist2dTest.cpp)
target_link_libraries(twist2dTest messagelib)
target_include_directories(twist2dTest PUBLIC include)

# Add mbotEncoders messages test
add_executable(mbotEncodersTest test/messages/mbotEncodersTest.cpp)
target_link_libraries(mbotEncodersTest messagelib)
target_include_directories(mbotEncodersTest PUBLIC include)

# Add lidarScan messages test
add_executable(lidarScanTest test/messages/lidarScanTest.cpp)
target_link_libraries(lidarScanTest messagelib)
target_include_directories(lidarScanTest PUBLIC include)

# Add pose messages test
add_executable(poseTest test/messages/poseTest.cpp)
target_link_libraries(poseTest messagelib)
target_include_directories(poseTest PUBLIC include)

# Add jointState messages test
add_executable(jointStateTest test/messages/jointStateTest.cpp)
target_link_libraries(jointStateTest messagelib linalg)
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

### COMBINED TESTS ###

add_executable(jointStatePublisher test/simpleFK/jointStatePublisher.cpp)
target_link_libraries(jointStatePublisher socketlib linalg utils messagelib)
target_include_directories(jointStatePublisher PRIVATE include)

add_executable(jointStateSubscription test/simpleFK/jointStateSubscription.cpp)
target_link_libraries(jointStateSubscription socketlib linalg utils messagelib)
target_include_directories(jointStateSubscription PRIVATE include)