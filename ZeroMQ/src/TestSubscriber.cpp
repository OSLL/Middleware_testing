#include "TestSubscriber.h"

TestSubscriber::TestSubscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_priority, int max_msg_size)
    : TestMiddlewareSub(topic, msgCount, prior, cpu_index, filename, topic_priority)
    , sock(context, zmq::socket_type::sub)
{
    for(auto &msg : _msgs)
	msg = zmq::message_t(max_msg_size+60);
    std::string str("tcp://127.0.0.1:56");
    str += std::to_string(topic.length());
    sock.connect(str);
    sock.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    int max_q = msgCount + 1000;
    sock.setsockopt(ZMQ_RCVHWM, &max_q, sizeof(max_q));
}

TestSubscriber::~TestSubscriber()
{
}

bool TestSubscriber::receive() {
    if(mcount >= _msgCount)
	return false;
    try{
        int cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        if(sock.recv(_msgs[mcount], zmq::recv_flags::none)){
	    _read_msg_time[mcount] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time;
	    _recieve_timestamps[mcount] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
	    ++mcount;
	    return true;
        }
    }
    catch(std::exception& e){}
    return false;
}

short TestSubscriber::get_id(zmq::message_t &msg) {
    nlohmann::json js = nlohmann::json::parse(std::string((char*)msg.data(), msg.size()));
    return js["id"];
}

unsigned long TestSubscriber::get_timestamp(zmq::message_t &msg) {
    nlohmann::json js = nlohmann::json::parse(std::string((char*)msg.data(), msg.size()));
    return js["timestamp"];
}
