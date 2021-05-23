#pragma once
#include <chrono>
#include <iostream>
#include <unistd.h>
#include "mqtt/client.h"
#include "../../interface/pub_interface.hpp"
#define ADDRESS     "tcp://localhost:1883"
#define QOS         1

class TestPublisher: public TestMiddlewarePub{
public:
    TestPublisher(std::string topic,  int msgCount, int prior, int cpu_index,
                  int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step, std::string &filename,
                  int topic_priority);


    unsigned long publish(short id, unsigned size) override;

    ~TestPublisher();

private:
    mqtt::async_client _client;
    mqtt::topic *_topic;
};
