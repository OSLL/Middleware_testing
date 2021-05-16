#pragma once
#include "../../interface/ping_pong_interface.hpp"
#include "mqtt/client.h"
#define ADDRESS     "tcp://localhost:1883"
#define QOS         1

class TestPingPongNode: public TestMiddlewarePingPong<std::string>{
public:
    TestPingPongNode(
            std::string &topic1, std::string &topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst);

    TestPingPongNode(std::string &topic1, std::string topic2, int msgCount, int prior,
                     int cpu_index, std::string &filename, int topic_priority, int msInterval,
                     int msgSizeMin, int msgSizeMax, int step, int before_step, bool isFirst);

    void init();

    bool receive() override;

    short get_id(std::string &msg) override;

    unsigned long get_timestamp(std::string &msg) override;

    void publish(short id, unsigned size) override;

private:
    mqtt::async_client _client_pub;
    mqtt::client _client_sub;
    mqtt::topic *_topic;
    short _last_id = -1;

};

