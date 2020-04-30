project(OpenDDS_DevGuide_Messenger CXX)
cmake_minimum_required(VERSION 3.8.2)

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

# Publisher
add_executable(publisher src/publisher/Publisher.cpp src/publisher/publisher.cpp include/Publisher.h)
OPENDDS_TARGET_SOURCES(publisher src/dds/Messenger.idl)
target_link_libraries(publisher ${opendds_libs} pthread)


# Subscriber
add_executable(subscriber src/subscriber/Subscriber.cpp src/subscriber/subscriber.cpp src/dds/DataReaderListenerImpl.cpp
        )
OPENDDS_TARGET_SOURCES(subscriber src/dds/Messenger.idl)
target_link_libraries(subscriber ${opendds_libs} pthread)
