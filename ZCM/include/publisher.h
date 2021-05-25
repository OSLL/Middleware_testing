#pragma once

#include "../../interface/pub_interface.hpp"
#include <zcm/zcm-cpp.hpp>

class TestPublisher : public TestMiddlewarePub
{
public:

    TestPublisher(std::string topic,  int msgCount, int prior, int cpu_index,
                  int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step,
                  std::string &filename, int topic_priority);

    unsigned long publish(short id, unsigned size) override;

    ~TestPublisher();

private:
    zcm::ZCM zcm;
};
