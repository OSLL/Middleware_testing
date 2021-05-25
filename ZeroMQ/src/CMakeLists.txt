cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cppzmq CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(ZeroMQ REQUIRED)
find_package(CPPZMQ REQUIRED)
find_package(Threads)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(
    ZeroMQ
    main.cpp
    TestPublisher.cpp
    TestSubscriber.cpp
    TestPingPong.cpp
    )
target_include_directories (ZeroMQ SYSTEM PRIVATE ${ZeroMQ_INCLUDE_DIR} ${CPPZMQ_INCLUDE_DIR})
target_link_libraries (ZeroMQ ${ZeroMQ_LIBRARIES} ${CPPZMQ_INCLUDE_DIR} ${CMAKE_THREAD_LIBS_INIT})
