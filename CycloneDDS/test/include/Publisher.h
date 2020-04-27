//
// Created by egor on 20.03.20.
//

#ifndef CYCLONE_TEST_PUBLISHER_H
#define CYCLONE_TEST_PUBLISHER_H

#include <string>
#include <chrono>


#include <Topic.h>
#include <Partisipant.h>
#include <Writer.h>
#include <QoS.h>
#include <Reader.h>


#include <pub_interface.hpp>


using namespace std::chrono;

template <class MsgType>
class Publisher: public TestMiddlewarePub {

public:
    Publisher(std::string &topic, int msg_count, int prior, int cpu_index, int min_msg_size, int max_msg_size,
              int step, int interval, int msgs_before_step, std::string& file_name, int topic_priority) :
            _topic(topic),
            _msInterval(interval),
            _msgCount(msg_count),
            _priority(prior),
            _cpu_index(cpu_index),
            _byteSizeMin(min_msg_size),
            _byteSizeMax(max_msg_size),
            _step(step),
            _msg_count_befor_step(msgs_before_step),
            TestMiddlewarePub(
                    topic, msg_count, prior, cpu_index, min_msg_size, max_msg_size,
                    step, interval, msgs_before_step, file_name, topic_priority)
    {};

    void create(dds_topic_descriptor topic_descriptor);

    unsigned long publish(short id, unsigned size) override;

    void cleanUp() override ;

private:
    MsgType _msg;
    int32_t _res_code;
    Writer<MsgType> _writer_entity ;
    Topic _topic_entity;
    Participant _participant_entity;

    std::string _topic;
    int _msInterval;
    int _msgCount;
    int _priority; //not stated
    int _cpu_index; //not stated
    int _byteSizeMin;
    int _byteSizeMax;
    int _step;
    int _msg_count_befor_step;

    dds_entity_t _participant;

};


#endif //CYCLONE_TEST_PUBLISHER_H
