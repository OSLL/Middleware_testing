cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cppzmq-sub CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(ZeroMQ REQUIRED)
find_package(CPPZMQ REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()
add_executable(
    sub
    sub.cpp
    )
target_include_directories (sub SYSTEM PRIVATE ${ZeroMQ_INCLUDE_DIR} ${CPPZMQ_INCLUDE_DIR})
target_link_libraries (sub ${ZeroMQ_LIBRARIES} ${CPPZMQ_INCLUDE_DIR})
add_executable(
    pub
    pub.cpp
    )
target_include_directories (pub SYSTEM PRIVATE ${ZeroMQ_INCLUDE_DIR} ${CPPZMQ_INCLUDE_DIR})
target_link_libraries (pub ${ZeroMQ_LIBRARIES} ${CPPZMQ_INCLUDE_DIR})
add_test(
  NAME
    sub
  COMMAND
    ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}/sub
  )
add_test(
  NAME
    pub
  COMMAND
    ${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}/pub
  )
