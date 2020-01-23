cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(cppzmq-sub CXX)

find_package(cppzmq)

enable_testing()
add_executable(
    sub
    sub.cpp
    )
add_executable(
    pub
    pub.cpp
    )
target_link_libraries(
    sub
    cppzmq
    )

target_link_libraries(
    pub
    cppzmq
    )
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
