//
// Created by egor on 20.03.20.
//

#ifndef CYCLONE_TEST_SUBSCRIBER_H
#define CYCLONE_TEST_SUBSCRIBER_H

#include <string>
#include <chrono>

#include <Topic.h>
#include <Partisipant.h>
#include <QoS.h>
#include <Reader.h>

#include <sub_interface.hpp>



using namespace std::chrono;

template <class MsgType>
class Subscriber: public TestMiddlewareSub<MsgType>{

public:
    Subscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_priority) :
            _topic(topic),
            _msgCount(msgCount),
            _priority(prior),
            _cpu_index(cpu_index),
            _filename(filename),
            TestMiddlewareSub<MsgType>(topic, msgCount, prior, cpu_index, filename, topic_priority)
    {};

    void create(dds_topic_descriptor topic_descriptor);

    short get_id(MsgType& msg) override;

    unsigned long get_timestamp(MsgType& msg) override;

    bool receive() override;

    void cleanUp() override;


private:

    std::string _topic;
    int _msgCount;
    int _priority; //def not stated
    int _cpu_index; //def not stated
    std::string _filename;

    MsgType _msg;
    int32_t _res_code;
    Reader<MsgType> _reader_entity ;
    Topic _topic_entity;
    Participant _participant_entity;


};


#endif //CYCLONE_TEST_SUBSCRIBER_H
