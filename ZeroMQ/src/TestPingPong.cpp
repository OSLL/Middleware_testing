#include "TestPingPong.h"
#include <iostream>

TestPingPong::TestPingPong(
            std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst) :
	    TestMiddlewarePingPong(topic1, topic2, msgCount, prior, cpu_index,
		        filename, topic_priority, msInterval, msgSize, isFirst)
            , psock(pcontext, zmq::socket_type::pub)
            , ssock(scontext, zmq::socket_type::sub)
	    , mcount(0)
{
    if(!isFirst)
        std::swap(_topic_name1, _topic_name2);

    _msgSizeMax = _msgSize;
    init();
}

TestPingPong::TestPingPong(
            std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSizeMin, int msgSizeMax, int step, int before_step, bool isFirst) :
	    TestMiddlewarePingPong(topic1, topic2, msgCount, prior, cpu_index,
		        filename, topic_priority, msInterval, msgSizeMin, msgSizeMax, step, before_step, isFirst)
            , psock(pcontext, zmq::socket_type::pub)
            , ssock(scontext, zmq::socket_type::sub)
	    , mcount(0)
{
    if(!isFirst)
        std::swap(_topic_name1, _topic_name2);

    init();

}

void TestPingPong::init()
{
    for(auto &msg : _msgs)
	    msg = std::make_shared<zmq::message_t>(zmq::message_t(_msgSizeMax+60));

    int max = 50000;
    int min = 1000;
    std::hash<std::string> hash_fn;

    auto port1 = ((unsigned int) hash_fn(_topic_name1) % max) + min;
    std::string str("tcp://127.0.0.1:");
    str += std::to_string(port1);
    std::cout << str << std::endl;
    psock.bind(str);
    int max_q = _msgCount + 1000;
    psock.setsockopt(ZMQ_SNDHWM, &max_q, sizeof(max_q));
    int dont_drop = 1;
    psock.setsockopt(ZMQ_XPUB_NODROP, &dont_drop, sizeof(dont_drop));

    str = "tcp://127.0.0.1:";
    auto port2 = ((unsigned int) hash_fn(_topic_name2) % max) + min;
    str += std::to_string(port2);
    std::cout << str << std::endl;
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
        auto *msg = new zmq::message_t();
        auto cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        if(ssock.recv(*msg, zmq::recv_flags::none)){
            std::shared_ptr<zmq::message_t> msg_ptr(msg);
            _read_msg_time[mcount] = std::chrono::duration_cast<std::chrono::nanoseconds>
                    (std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time;
            write_received_msg(msg_ptr);
            ++mcount;
	    return true;
        }
    }
    catch(std::exception& e){}
    return false;
}

short TestPingPong::get_id(std::shared_ptr<zmq::message_t> &msg) {
    nlohmann::json js = nlohmann::json::parse(std::string((char*)msg->data(), msg->size()));
    return js["id"];
}

unsigned long TestPingPong::get_timestamp(std::shared_ptr<zmq::message_t> &msg) {
    nlohmann::json js = nlohmann::json::parse(std::string((char*)msg->data(), msg->size()));
    return js["timestamp"];
}

void TestPingPong::publish(short id, unsigned size) {
    std::string data(size, 'a');
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    //std::cout<<_isFirst<< "Start sending " << id << std::endl;
    nlohmann::json jmsg;
    jmsg["id"] = id;
    jmsg["timestamp"] = cur_time;
    jmsg["msg"] = data;
    std::string jstr = jmsg.dump();
    cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    psock.send((const void*)jstr.c_str(), (size_t)jstr.length());
    _write_msg_time[id] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time;
    //std::cout<<_isFirst<< "End sending " << id << std::endl;
}
