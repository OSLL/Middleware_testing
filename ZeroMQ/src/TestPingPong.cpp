#include "TestPingPong.h"

TestPingPong::TestPingPong(
            std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst) :
	    TestMiddlewarePingPong(topic1, topic2, msgCount, prior, cpu_index,
		        filename, topic_priority, msInterval, msgSize, isFirst)
            , psock(pcontext, zmq::socket_type::pub)
            , ssock(scontext, zmq::socket_type::sub)
{
    if(!isFirst)
        std::swap(_topic_name1, _topic_name2);

    for(auto &msg : _msgs)
	msg = zmq::message_t(msgSize+60);
    std::string str("tcp://127.0.0.1:56");
    str += std::to_string(_topic_name1.length());
    psock.bind(str);
    int max_q = msgCount + 1000;
    psock.setsockopt(ZMQ_SNDHWM, &max_q, sizeof(max_q));
    int dont_drop = 1;
    psock.setsockopt(ZMQ_XPUB_NODROP, &dont_drop, sizeof(dont_drop));
    
    str = "tcp://127.0.0.1:56";
    str += std::to_string(_topic_name2.length());
    ssock.connect(str);
    ssock.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    ssock.setsockopt(ZMQ_RCVHWM, &max_q, sizeof(max_q));
}

TestPingPong::~TestPingPong()
{
}

bool TestPingPong::receive() {
    if(mcount >= _msgCount)
	return false;
    try{
        int cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        if(ssock.recv(_msgs[mcount], zmq::recv_flags::none)){
	    _recieve_timestamps[mcount] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
	    ++mcount;
	    return true;
        }
    }
    catch(std::exception& e){}
    return false;
}

short TestPingPong::get_id(zmq::message_t &msg) {
    nlohmann::json js = nlohmann::json::parse(std::string((char*)msg.data(), msg.size()));
    return js["id"];
}

unsigned long TestPingPong::get_timestamp(zmq::message_t &msg) {
    nlohmann::json js = nlohmann::json::parse(std::string((char*)msg.data(), msg.size()));
    return js["timestamp"];
}

void TestPingPong::publish(short id, unsigned size) {
    std::string data(size, 'a');
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    
    nlohmann::json jmsg;
    jmsg["id"] = id;
    jmsg["timestamp"] = cur_time;
    jmsg["msg"] = data;
    std::string jstr = jmsg.dump();
    psock.send((const void*)jstr.c_str(), (size_t)jstr.length());
}
