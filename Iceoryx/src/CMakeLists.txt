cmake_minimum_required(VERSION 2.2)

project(IceoryxTest)

add_executable(PubSub pubsub.cpp)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")


target_link_libraries(PubSub
       	iceoryx_posh iceoryx_utils
       	iceoryx_posh_roudi 
	iceoryx_platform 
	iceoryx_introspection
	pthread
	rt
)

