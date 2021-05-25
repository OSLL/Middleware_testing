cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(endurox CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(GCC_COVERAGE_COMPILE_FLAGS "-latmiclt -latmi -lubf -lnstd -lrt -ldl -lm -fpermissive")
add_executable(
    Endurox
    main.cpp
    TestPublisher.cpp
    TestSubscriber.cpp
    TestPingPong.cpp
    )

get_target_property(TEMP Endurox COMPILE_FLAGS)
if(TEMP STREQUAL "TEMP-NOTFOUND")
  SET(TEMP "") # Set to empty string
else()
  SET(TEMP "${TEMP} ") # A space to cleanly separate from existing content
endif()
# Append our values
SET(TEMP "${TEMP}${GCC_COVERAGE_COMPILE_FLAGS}" )
set_target_properties(Endurox PROPERTIES COMPILE_FLAGS "${TEMP}")

target_link_libraries (Endurox ${ZeroMQ_LIBRARIES} ${CPPZMQ_INCLUDE_DIR})
target_link_libraries (Endurox "${GCC_COVERAGE_COMPILE_FLAGS}")
