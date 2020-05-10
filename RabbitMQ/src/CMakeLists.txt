cmake_minimum_required(VERSION 2.2)

project(RabbitmqTest)

add_executable(pub pub.cpp)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

find_package(SimpleAmqpClient REQUIRED)

add_executable(sub sub.cpp)

target_sources(pub PUBLIC msg.cpp)
target_sources(sub PUBLIC msg.cpp)

target_link_libraries(pub SimpleAmqpClient)
target_link_libraries(sub SimpleAmqpClient)
