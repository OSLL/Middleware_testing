cmake_minimum_required(VERSION 3.16)
project(open62541 LANGUAGES CXX)
set_source_files_properties(filename.c PROPERTIES LANGUAGE CXX )
set(CMAKE_CXX_STANDARD 17)

LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/open62541)

option(WITH_EVENTS "Build publisher and subscriber using event system" ON)

if (WITH_EVENTS)
    add_definitions(-DUSE_EVENTS)
endif()
add_executable(server server.cpp)
add_executable(node open62541_node.cpp)


target_link_libraries(server open62541)
target_link_libraries(node open62541)
