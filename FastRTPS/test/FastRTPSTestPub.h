#include <string>
#include "test_interface.hpp"
#include "TestPublisher.h"
#include "TestSubscriber.h"
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/attributes/ParticipantAttributes.h>

#include <thread>
#include <chrono>

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

class FastRTPSTestPub : public TestMiddlewarePub<TestPublisher>
{

public:
    explicit FastRTPSTestPub(std::string topic,  int msgCount=0, int prior = -1, int cpu_index = -1,
            int min_msg_size=0, int max_msg_size=64000, int step=0, int interval = 0, int msgs_before_step = 100) : 
    TestMiddlewarePub(topic, msgCount, prior, cpu_index, min_msg_size, max_msg_size, step, interval, msgs_before_step)
    {}

    virtual TestPublisher* createPublisher(std::string topic);

    virtual void publish(std::string &msg);

//    virtual void setQoS(std::string filename);
};
