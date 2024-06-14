cmake_minimum_required(VERSION 3.0)

project(mros)

set(CMAKE_CXX_STANDARD 11)

find_library(PTHREAD pthread)

add_library(smemlib STATIC src/smem/sbuffer.cpp
                           src/smem/smutex.cpp)
target_include_directories(smemlib PUBLIC include)
target_link_libraries(smemlib pthread)

add_library(socketlib STATIC src/socket/client.cpp
                             src/socket/server.cpp)
target_include_directories(socketlib PUBLIC include)

add_library(mros STATIC src/mros/core.cpp
                        src/mros/publisher.cpp
                        src/mros/subscriber.cpp)
target_include_directories(mros PUBLIC include)
target_link_libraries(mros socketlib smemlib)

# Add core test
add_executable(core_test test/mros_test/core_test.cpp)
target_link_libraries(core_test mros)

# Add publisher test
add_executable(publisher_test test/mros_test/publisher_test.cpp)
target_link_libraries(publisher_test mros)

# Add subscriber test
add_executable(subscriber_test test/mros_test/subscriber_test.cpp)
target_link_libraries(subscriber_test mros)

# Add socket client test
add_executable(client_test test/socket_test/client_test.cpp)
target_link_libraries(client_test socketlib)

# Add socket server test
add_executable(server_test test/socket_test/server_test.cpp)
target_link_libraries(server_test socketlib)

# Add socket client test
add_executable(client_noblock_test test/socket_test/client_noblock_test.cpp)
target_link_libraries(client_noblock_test socketlib)

# Add socket server test
add_executable(server_noblock_test test/socket_test/server_noblock_test.cpp)
target_link_libraries(server_noblock_test socketlib)

# Add smem read test
add_executable(smemReadTest test/smem_test/smemReadTest.cpp)
target_link_libraries(smemReadTest smemlib)

# Add smem write test
add_executable(smemWriteTest test/smem_test/smemWriteTest.cpp)
target_link_libraries(smemWriteTest smemlib)

# Add pose2d messages test
add_executable(pose2dTest test/messages_test/pose2dTest.cpp)
target_include_directories(pose2dTest PUBLIC include)

# Add twist2d messages test
add_executable(twist2dTest test/messages_test/twist2dTest.cpp)
target_include_directories(twist2dTest PUBLIC include)

# Add mbotEncoders messages test
add_executable(mbotEncodersTest test/messages_test/mbotEncodersTest.cpp)
target_include_directories(mbotEncodersTest PUBLIC include)

# Add lidarScan messages test
add_executable(lidarScanTest test/messages_test/lidarScanTest.cpp)
target_include_directories(lidarScanTest PUBLIC include)