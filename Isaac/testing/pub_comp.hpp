#pragma once
#include "pub_interface.hpp"

class PublisherComponent: public TestMiddlewarePub{
public:
    PublisherComponent(std::string topic,  int msgCount, int prior, int cpu_index,
            int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step,
            std::string filename, int topic_priority) : 
        TestMiddlewarePub(topic, msgCount, prior, cpu_index,
        min_msg_size, max_msg_size, step, interval, msgs_before_step,
        filename, topic_priority)
    {
    }

    unsigned long publish(short id, unsigned size) override{ return 0;}

    void write_proc_msg(int id, unsigned long time){
        _write_msg_time[id] = time;
    }

    int get_msg_count(){ return _msgCount;}

    int get_min_msg_size(){ return _byteSizeMin;}

    int get_max_msg_size(){ return _byteSizeMax;}

    int get_step(){ return _step;}

    int get_before_step(){ return _msg_count_befor_step;}
    
    std::vector<unsigned long>& get_proc_times(){return _write_msg_time;}
};
