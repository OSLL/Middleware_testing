#pragma once
#include "mqtt/client.h"
#include "../../interface/sub_interface.hpp"
#define ADDRESS     "tcp://localhost:1883"
#define QOS         1

class TestSubscriber: public TestMiddlewareSub<std::string>{
public:
    TestSubscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_priority);

    bool receive() override;

    short get_id(std::string &msg) override;

    unsigned long get_timestamp(std::string &msg) override;

private:
    mqtt::client _client;
    short _last_id = -1;
};
