#include "TestSubscriber.h"

TestSubscriber::TestSubscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_priority, int max_msg_size)
    : TestMiddlewareSub(topic, msgCount, prior, cpu_index, filename, topic_priority)
    , sock(context, zmq::socket_type::sub)
{
    for(auto &msg : _msgs)
	    msg = std::make_shared<zmq::message_t>(zmq::message_t(max_msg_size+60));
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
        auto *msg = new zmq::message_t();
        int cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        if(sock.recv(*msg, zmq::recv_flags::none)){
            std::shared_ptr<zmq::message_t> msg_ptr(msg);
            write_received_msg(msg_ptr, std::chrono::duration_cast<std::chrono::nanoseconds>
                    (std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time);
            ++mcount;
            return true;
        }
    }
    catch(std::exception& e){}
    return false;
}

short TestSubscriber::get_id(std::shared_ptr<zmq::message_t> &msg) {
    nlohmann::json js = nlohmann::json::parse(std::string((char*)msg->data(), msg->size()));
    return js["id"];
}

unsigned long TestSubscriber::get_timestamp(std::shared_ptr<zmq::message_t> &msg) {
    nlohmann::json js = nlohmann::json::parse(std::string((char*)msg->data(), msg->size()));
    return js["timestamp"];
}
