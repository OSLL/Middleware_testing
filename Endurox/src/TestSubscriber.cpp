#include "TestSubscriber.h"

TestSubscriber* sub;

static void handler(char* data, long len, long flags){
	sub->msgHandler(data, len, flags);
}

TestSubscriber::TestSubscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_priority, int max_msg_size)
    : TestMiddlewareSub(topic, msgCount, prior, cpu_index, filename, topic_priority)
{
	//handle_func handler = reinterpret_cast<handle_func>(std::bind(&TestSubscriber::msgHandler, this, _1, _2, _3));
	for(int i=0; i<_msgs.size(); ++i)
		_msgs[i] = new char[max_msg_size+200];
	sub = this;
	//std::function<void(char*,long,long)> handler = [sub](char* data, long len, long flag) -> void {sub->msgHandler(data, len, flag);};
	tpsetunsol(handler);
}

TestSubscriber::~TestSubscriber()
{
	tpterm();
}

bool TestSubscriber::receive() {
	return tpchkunsol();
}

short TestSubscriber::get_id(char* &msg) {
    try{
    nlohmann::json js = nlohmann::json::parse(msg);
    return js["id"];
    }
    catch(...){
    return 0;
    }
}

unsigned long TestSubscriber::get_timestamp(char* &msg) {
    try{
    nlohmann::json js = nlohmann::json::parse(msg);
    return js["timestamp"];
    }
    catch(...){
    return 0;
    }
}

void TestSubscriber::msgHandler(char* data, long len, long flags){
    _read_msg_time[n_received] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    strcpy(_msgs[n_received], data);
    _read_msg_time[n_received] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - _read_msg_time[n_received];
    _recieve_timestamps[n_received] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    ++n_received;
}

