cmake_minimum_required(VERSION 3.16)
project(open62541 LANGUAGES CXX)
set_source_files_properties(filename.c PROPERTIES LANGUAGE CXX )
set(CMAKE_CXX_STANDARD 17)

LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/open62541)

#add_executable(client_pub client_pub.cpp)
#add_executable(client_sub client_sub.cpp)
add_executable(server server.cpp)
add_executable(node open62541_node.cpp)


#target_link_libraries(client_pub open62541)
#target_link_libraries(client_sub open62541)
target_link_libraries(server open62541)
target_link_libraries(node open62541)
