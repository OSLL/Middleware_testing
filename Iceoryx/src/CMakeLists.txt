cmake_minimum_required(VERSION 2.2)

project(IceoryxTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_COMPILER "/usr/bin/g++-8")
add_executable(PubSub pubsub.cpp)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

#find_package(Iceoryx REQUIRED)

target_link_libraries(PubSub
       	iceoryx_posh iceoryx_utils
       	iceoryx_posh_roudi 
	iceoryx_platform 
	iceoryx_introspection
	pthread
	rt
)

