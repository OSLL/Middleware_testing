cmake_minimum_required(VERSION 3.16)
project(open62541 LANGUAGES CXX)
set_source_files_properties(filename.c PROPERTIES LANGUAGE CXX )
set(CMAKE_CXX_STANDARD 98)

LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/open62541)

add_executable(server server.cpp)
add_executable(client client.cpp)

target_link_libraries(server open62541)
target_link_libraries(client open62541)
