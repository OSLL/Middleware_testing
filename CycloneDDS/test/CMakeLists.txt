CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT(cyclone_test)

set( CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#cmake_policy(SET CMP0054 OLD)

find_package(CycloneDDS
        REQUIRED PATHS "/home/egor/.Lib/cyclone_dds"
        COMPONENTS idlc)

include_directories(include
                    ${CMAKE_CURRENT_BINARY_DIR}
                    /home/egor/.Lib/cyclone_dds/include
                    ../../interface
        )


SET(INC
        include/Publisher.h
        include/Subscriber.h
        include/Partisipant.h
        include/Topic.h
        include/Writer.h
        include/Reader.h
        include/QoS.h
        )

set(SRC
        ${CMAKE_CURRENT_BINARY_DIR}/TypeData.c
        #CyclonePublisher.cpp)
        )

idlc_generate(${PROJECT_NAME}_lib "src/dds/TypeData.idl")

add_executable(publisher ${SRC} ${INC} src/publisher/publisher.cpp src/publisher/Publisher.cpp)
add_executable(subscriber ${SRC} ${INC} src/subscriber/subscriber.cpp src/subscriber/Subscriber.cpp)

target_link_libraries(publisher ${PROJECT_NAME}_lib CycloneDDS::ddsc )
target_link_libraries(subscriber ${PROJECT_NAME}_lib CycloneDDS::ddsc )