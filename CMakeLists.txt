cmake_minimum_required(VERSION 3.0)

project(mros)

set(CMAKE_CXX_STANDARD 11)

add_library(socketlib STATIC src/socket/client.cpp
                             src/socket/server.cpp)
target_include_directories(socketlib PUBLIC include)

add_library(mros STATIC src/mros/core.cpp
                        src/mros/publisher.cpp
                        src/mros/subscriber.cpp)
target_include_directories(mros PUBLIC include)
target_link_libraries(mros socketlib)

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