cmake_minimum_required(VERSION 3.16)
project(open62541 LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 17)

LINK_DIRECTORIES(${CMAKE_BINARY_DIR}/open62541)

option(WITH_EVENTS "Build publisher and subscriber using event system" ON)

if (WITH_EVENTS)
    add_definitions(-DUSE_EVENTS)
endif()
#add_executable(server server.cpp DataType.c)
#add_executable(node open62541_node.cpp)

#add_executable(sub ver_pubsub/sub.cpp)
#add_executable(pub ver_pubsub/pubsub.c DataType.c ver_pubsub/pub.cpp)

add_executable(publish_realtime tsn_testing/publisher.cpp ./tsn_testing/bufmalloc.c)

# target_link_libraries(server pthread open62541)
# target_link_libraries(node pthread open62541)
# target_link_libraries(sub pthread open62541)
# target_link_libraries(pub pthread open62541)
target_link_libraries(publish_realtime pthread rt open62541)
