#include <thread>

#include <nlohmann/json.hpp>
#include "TestPublisher.h"


#include <iostream>

TestPublisher::TestPublisher(std::string topic,  int msgCount, int prior, int cpu_index,
                  int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step,
		  std::string &filename, int topic_priority)
    : TestMiddlewarePub(topic, msgCount, prior, cpu_index, min_msg_size, max_msg_size, step, interval, msgs_before_step, filename, topic_priority)
    , sock(context, zmq::socket_type::pub)
{
    std::string str("tcp://127.0.0.1:56");
    str += std::to_string(topic.length());

    sock.bind(str);
    int max_q = msgCount + 1000;
    sock.setsockopt(ZMQ_SNDHWM, &max_q, sizeof(max_q));
    int dont_drop = 1;
    sock.setsockopt(ZMQ_XPUB_NODROP, &dont_drop, sizeof(dont_drop));
}

TestPublisher::~TestPublisher()
{
}

unsigned long TestPublisher::publish(short id, unsigned size) {
    std::string data(size, 'a');
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    
    nlohmann::json jmsg;
    jmsg["id"] = id;
    jmsg["timestamp"] = cur_time;
    jmsg["msg"] = data;
    std::string jstr = jmsg.dump();
    cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    sock.send((const void*)jstr.c_str(), (size_t)jstr.length());
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time;
}

