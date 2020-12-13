#include "../include/ping_pong.h"

#include <iostream>

TestPingPong::TestPingPong(
        std::string &topic1, std::string topic2, int msgCount, int prior,
        int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst) :
        TestMiddlewarePingPong(topic1, topic2, msgCount, prior, cpu_index,
                               filename, topic_priority, msInterval, msgSize, isFirst),
        zcm("ipc://test")
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
                               filename, topic_priority, msInterval, msgSizeMin, msgSizeMax, step, before_step, isFirst),
        zcm("ipc://test")
{
    if(!isFirst)
        std::swap(_topic_name1, _topic_name2);

    init();

}

void TestPingPong::init()
{
    if (!zcm.good())
        std::cout << "Error in creating zcm object!\n";
    if(_topic_name1[0]=='/')
        _topic_name1.erase(0, 1);
    if(_topic_name2[0]=='/')
        _topic_name2.erase(0, 1);
    std::cout << _topic_name1 << " "<<_topic_name2 <<std::endl;
    zcm.subscribe(_topic_name2,  &TestPingPong::handleMessage, this);
    zcm.start();
}


TestPingPong::~TestPingPong()
{
    zcm.stop();
}

bool TestPingPong::receive() {
    if (isReceived){
        isReceived = false;
        return true;
    }
    return false;
}

short TestPingPong::get_id(msg_t &msg) {
    return msg.id;
}

unsigned long TestPingPong::get_timestamp(msg_t &msg) {
    return msg.timestamp;
}

void TestPingPong::handleMessage(const zcm::ReceiveBuffer *rbuf, const std::string &chan, const msg_t *msg) {
    unsigned long timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>
            (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    msg_t rec_msg;
    rec_msg.id = msg->id;
    std::cout << rec_msg.id << std::endl;
    rec_msg.timestamp = msg->timestamp;
    write_received_msg(rec_msg);
    _read_msg_time[get_id(rec_msg)] = std::chrono::duration_cast<std::chrono::nanoseconds>
            (std::chrono::high_resolution_clock::now().time_since_epoch()).count() - rbuf->recv_utime;
    isReceived = true;
}

void TestPingPong::publish(short id, unsigned size) {
    std::string data(size, 'a');
    msg_t msg;
    msg.id = id;
    std::cout << id << std::endl;
    msg.str = data;
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    msg.timestamp = cur_time;
    int status = zcm.publish(_topic_name1, &msg);
    if(status < 0)
        std::cout << "Unsuccessful publishing: id "<< id << std::endl;
    zcm.flush();
    _write_msg_time[id] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time;
}

