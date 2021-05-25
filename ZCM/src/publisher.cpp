#include <thread>
#include <iostream>
#include "../include/publisher.h"
#include "../include/msg_t.hpp"

TestPublisher::TestPublisher(std::string topic,  int msgCount, int prior, int cpu_index,
                             int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step,
                             std::string &filename, int topic_priority)
        : TestMiddlewarePub(topic, msgCount, prior, cpu_index, min_msg_size, max_msg_size, step, interval,
                            msgs_before_step, filename, topic_priority),
                            zcm("ipc://test")
{
    if(_topic_name[0]=='/')
        _topic_name.erase(0, 1);
    if (!zcm.good())
        std::cout << "Error in creating zcm object!\n";
    std::cout <<_topic_name <<"\n";
    zcm.start();
    msg_t msg;
    zcm.publish(_topic_name, &msg); // msg sending before starting the test
}

unsigned long TestPublisher::publish(short id, unsigned size) {
    std::string data(size, 'a');
    msg_t msg;
    msg.id = id;
    //std::cout << id << std::endl;
    msg.str = data;
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    msg.timestamp = cur_time;
    int status = -1;
    while (status < 0) {
        status = zcm.publish(_topic_name, &msg);
        //std::cout << "Unsuccessful publishing: id " << id << std::endl;
        std::this_thread::sleep_for(std::chrono::microseconds(5));
    }
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time;
}

TestPublisher::~TestPublisher() {
    zcm.stop();
}


