#include <thread>

#include <nlohmann/json.hpp>
#include "TestPublisher.h"


#include <iostream>

TestPublisher::TestPublisher(std::string topic,  int msgCount, int prior, int cpu_index,
                  int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step,
		  std::string &filename, int topic_priority)
    : TestMiddlewarePub(topic, msgCount, prior, cpu_index, min_msg_size, max_msg_size, step, interval, msgs_before_step, filename, topic_priority)
{
    if ((strbuf = tpalloc("STRING", NULL, max_msg_size+200)) == NULL) {
         fprintf(stderr, "Failed to alloc the buffer - %s\n",
                 tpstrerror(tperrno));
    }

}

TestPublisher::~TestPublisher()
{
	tpterm();
}

unsigned long TestPublisher::publish(short id, unsigned size) {
    std::string data(size, 'a');
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    
    nlohmann::json jmsg;
    jmsg["id"] = id;
    jmsg["timestamp"] = cur_time;
    jmsg["msg"] = data;
    std::string jstr = jmsg.dump();
    strcpy(strbuf, jstr.c_str());
    cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    //tpbroadcast(NULL, NULL, NULL, strbuf, 0, TPSIGRSTRT);
    if (tpbroadcast(NULL, /*(char*)_topic_name1.c_str()*/NULL, NULL, strbuf, 0, TPSIGRSTRT) == -1){
                fprintf(stderr, "Failed to broadcast - %s\n",
                        tpstrerror(tperrno));
    }
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time;
}

