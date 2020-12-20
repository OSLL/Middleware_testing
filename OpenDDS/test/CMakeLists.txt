project(OpenDDS_DevGuide_Messenger CXX)
cmake_minimum_required(VERSION 3.8.5)

find_package(OpenDDS REQUIRED)

set(CMAKE_CXX_COMPILER ${OPENDDS_COMPILER})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(opendds_libs
  OpenDDS::Dcps # Core OpenDDS Library
)

include_directories(../../interface
                    include
        )



add_executable(opendds_node src/ping_pong/ping_pong.cpp src/subscriber/Subscriber.cpp src/dds/DataReaderListenerImpl.cpp
        src/publisher/Publisher.cpp src/opendds_node.cpp)
OPENDDS_TARGET_SOURCES(ping_pong src/dds/Messenger.idl)
target_link_libraries(ping_pong ${opendds_libs} pthread)
