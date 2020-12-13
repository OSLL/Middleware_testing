#include "../include/subscriber.h"


TestSubscriber::TestSubscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_priority, int max_msg_size)
        : TestMiddlewareSub(topic, msgCount, prior, cpu_index, filename, topic_priority),
        zcm("ipc://test")
{
    if (!zcm.good())
        std::cout << "Error in creating zcm object!\n";
    if(_topic_name[0]=='/')
        _topic_name.erase(0, 1);
    zcm.subscribe(_topic_name,  &TestSubscriber::handleMessage, this);
    zcm.start();
    std::cout << "Subscriber started!\n";

}

TestSubscriber::~TestSubscriber()
{
    zcm.stop();
}

bool TestSubscriber::receive() {
    if (isReceived){
        isReceived = false;
        return true;
    }
    return false;
}

short TestSubscriber::get_id(msg_t &msg) {
    return msg.id;
}

unsigned long TestSubscriber::get_timestamp(msg_t &msg) {
    return msg.timestamp;
}

void TestSubscriber::handleMessage(const zcm::ReceiveBuffer *rbuf, const std::string &chan, const msg_t *msg) {
    unsigned long proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>
            (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    msg_t rec_msg;
    rec_msg.id = msg->id;
    //std::cout << msg->id << std::endl;
    rec_msg.timestamp = msg->timestamp;
    proc_time = proc_time - std::chrono::duration_cast<std::chrono::nanoseconds>
            (std::chrono::microseconds(rbuf->recv_utime)).count();
    write_received_msg(rec_msg, proc_time);
    isReceived = true;
}
