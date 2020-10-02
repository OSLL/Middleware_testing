cmake_minimum_required(VERSION 3.0)

project(S2OPC_Pub)

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

#add_executable(YamiPubSub pubsub.cpp)
add_executable(Pub pubsub.cpp pub.cpp server.c pubsub.c helpers.c)

#find_library(YAMICPP yamicpp yamicpp.a libyamicpp.a REQUIRED NO_DEFAULT_PATH)

#find_library(YAMICPP yamicore yamicore.a libyamicore.a REQUIRED NO_DEFAULT_PATH)

set(CMAKE_CXX_FLAGS -fpermissive)

target_link_libraries(Pub
	s2opc_clientserver
	s2opc_clientserver-loader-embedded
        s2opc_pubsub
        s2opc_common
        s2opc_clientwrapper
	s2opc_clientserver-xml-loaders-expat
        pthread
        mbedcrypto
        mbedx509
        expat
	)

add_custom_command(TARGET Pub POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy 
    ${CMAKE_CURRENT_SOURCE_DIR}/pubsub_server.xml 
    ${CMAKE_CURRENT_BINARY_DIR}/pubsub_server.xml)
