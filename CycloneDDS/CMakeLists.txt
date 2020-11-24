CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT(cyclone_test)

set( CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CYCLONEDDS_DIR
        /home/andrew/work/middleware_project/dependences/cyclonedds/)

find_package(CycloneDDS
        REQUIRED PATHS ${CYCLONEDDS_DIR}/install/lib/cmake/CycloneDDS
        COMPONENTS idlc)

include_directories(include
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${CYCLONEDDS_DIR}install/lib/cmake/CycloneDDS
                    ../interface
        )


SET(INC
        include/Publisher.h
        include/PingPong.h
        include/Subscriber.h
        include/Partisipant.h
        include/Topic.h
        include/Writer.h
        include/Reader.h
        include/QoS.h
        )

set(SRC
        ${CMAKE_CURRENT_BINARY_DIR}/TypeData.c
        src/publisher/Publisher.cpp
        src/subscriber/Subscriber.cpp
        src/ping_pong/PingPong.cpp
        )

idlc_generate(${PROJECT_NAME}_lib "src/dds/TypeData.idl")

add_executable(cyclonedd_node ${SRC} ${INC} src/cyclonedds_node.cpp)

target_link_libraries(cyclonedd_node ${PROJECT_NAME}_lib CycloneDDS::ddsc pthread)