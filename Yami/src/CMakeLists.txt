cmake_minimum_required(VERSION 3.0)

project(YAMI4)

if(CMAKE_COMPILER_IS_GNUCXX)
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.3.0)
        message(FATAL_ERROR "Compiler version is lower than required. Required >=8.3.0")
    endif()
endif()

include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(-std=c++17 SUPPORTS_CXX17)
    if(NOT SUPPORTS_CXX17)
        message(FATAL_ERROR "Compiler doesn't support C++17")
    endif()
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(YamiPubSub pubsub.cpp)

find_library(YAMICPP yamicpp yamicpp.a libyamicpp.a REQUIRED NO_DEFAULT_PATH)

find_library(YAMICPP yamicore yamicore.a libyamicore.a REQUIRED NO_DEFAULT_PATH)

target_link_libraries(YamiPubSub
	yamicpp
	yamicore
	pthread
	)


