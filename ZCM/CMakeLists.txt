cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(zcm_node CXX)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

include(GenerateZcmType)
find_package(Threads)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SRC_FILES
        ./src/main.cpp
        ./src/ping_pong.cpp
        ./src/publisher.cpp
        ./src/subscriber.cpp
        )

ZCM_GEN(DataType/msg_t)

add_executable(
        zcm_node
        ${SRC_FILES}
        ${outsources}
)

target_link_libraries(zcm_node zcm pthread)