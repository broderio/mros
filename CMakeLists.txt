cmake_minimum_required(VERSION 3.0)

project(mros)

set(CMAKE_CXX_STANDARD 11)

find_library(PTHREAD pthread)

add_library(utils STATIC src/utils.cpp)
target_include_directories(utils PRIVATE include)

add_library(smemlib STATIC src/smem/squeue.cpp
                           src/smem/smutex.cpp
                           src/smem/smem.cpp)
target_include_directories(smemlib PRIVATE include)
target_link_libraries(smemlib pthread utils)

add_library(socketlib STATIC src/socket/tcp/client.cpp
                             src/socket/tcp/server.cpp
                             src/socket/udp/client.cpp
                             src/socket/udp/server.cpp)
target_include_directories(socketlib PRIVATE include)
target_link_libraries(socketlib utils)

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

### SMEM TESTS ###

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
target_include_directories(pose2dTest PUBLIC include)

# Add twist2d messages test
add_executable(twist2dTest test/messages/twist2dTest.cpp)
target_include_directories(twist2dTest PUBLIC include)

# Add mbotEncoders messages test
add_executable(mbotEncodersTest test/messages/mbotEncodersTest.cpp)
target_include_directories(mbotEncodersTest PUBLIC include)

# Add lidarScan messages test
add_executable(lidarScanTest test/messages/lidarScanTest.cpp)
target_include_directories(lidarScanTest PUBLIC include)

# Add pose messages test
add_executable(poseTest test/messages/poseTest.cpp)
target_include_directories(poseTest PUBLIC include)
