#pragma once
#include <zcm/zcm-cpp.hpp>
#include <atomic>
#include "../../interface/ping_pong_interface.hpp"
#include "../include/msg_t.hpp"

class TestPingPong : public TestMiddlewarePingPong<msg_t>
{
public:

    TestPingPong(
            std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst);

    TestPingPong(
            std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority,
            int msInterval, int msgSizeMin, int msgSizeMax, int step,
            int before_step, bool isFirst);

    void init();

    virtual ~TestPingPong();

    bool receive();

    short get_id(msg_t &msg);

    unsigned long get_timestamp(msg_t &msg);

    void publish(short id, unsigned size);
private:
    void handleMessage(const zcm::ReceiveBuffer *rbuf,
                       const std::string &chan,
                       const msg_t *msg);

    std::atomic_bool isReceived = false;

    zcm::ZCM zcm;

};
