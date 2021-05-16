cmake_minimum_required (VERSION 2.6)
set(CMAKE_MODULE_PATH ../cmake)
project (mqtt-testing)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

ADD_EXECUTABLE(MQTTClient
        ./src/paho_node.cpp
        ./src/publisher.cpp
        ./src/subscriber.cpp
        ./src/ping_pong.cpp)

TARGET_LINK_LIBRARIES(MQTTClient paho-mqttpp3 paho-mqtt3c pthread)

