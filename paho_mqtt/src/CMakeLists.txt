cmake_minimum_required (VERSION 2.6)
set(CMAKE_MODULE_PATH ../cmake)
project (mqtt-testing)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set (THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

ADD_EXECUTABLE(MQTTClient_publish paho_node.cpp publisher.cpp subscriber.cpp)

TARGET_LINK_LIBRARIES(MQTTClient_publish paho-mqttpp3 paho-mqtt3c)

