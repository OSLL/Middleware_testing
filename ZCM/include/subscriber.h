#pragma once
#include <atomic>
#include <zcm/zcm-cpp.hpp>
#include "../../interface/sub_interface.hpp"
#include "../include/msg_t.hpp"

class TestSubscriber : public TestMiddlewareSub<msg_t>
{
public:

    TestSubscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_prior, int max_msg_size);

    virtual ~TestSubscriber();

    bool receive();

    short get_id(msg_t &msg);

    unsigned long get_timestamp(msg_t &msg);

private:
    void handleMessage(const zcm::ReceiveBuffer *rbuf,
                       const std::string &chan,
                       const msg_t *msg);

    std::atomic_bool isReceived = false;

    zcm::ZCM zcm;
};

