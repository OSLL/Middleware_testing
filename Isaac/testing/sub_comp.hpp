#pragma once
#include "sub_interface.hpp"
#include "msg.h"

class SubscriberComponent: public TestMiddlewareSub<Msg>{
public: 
    SubscriberComponent(std::string topic, int msgCount, int prior,
            int cpu_index, std::string filename, int topic_priority) :
        TestMiddlewareSub(topic, msgCount, prior,
        cpu_index, filename, topic_priority)
        {
        }

    bool receive() override{ return true;}

    short get_id(Msg& msg) override { return msg.id;}

    unsigned long get_timestamp(Msg& msg) override{ return msg.timestamp;}

    virtual ~SubscriberComponent(){}
};
