#pragma once
#include "ping_pong_interface.hpp"
#include "msg.h"

class PingPongComponent: public TestMiddlewarePingPong<Msg>{
public:
    PingPongComponent(std::string topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string filename, int topic_priority, int msInterval,
            int msgSizeMin, int msgSizeMax, int step, int before_step, bool isFirst) :
        TestMiddlewarePingPong(topic1, topic2, msgCount, prior,
            cpu_index, filename, topic_priority, msInterval,
            msgSizeMin, msgSizeMax, step, before_step, isFirst)
    {
    } 

    bool receive() override {return true;}

    short get_id(Msg& msg){return msg.id;}

    unsigned long get_timestamp(Msg& msg){return msg.timestamp;}

    void publish(short id, unsigned size){}

    void write_proc_time(int id, unsigned long proc){
        _write_msg_time[id] = proc;
    }

    void write_read_time(int id, unsigned long read){
        _read_msg_time[id] = read;
    }
};
