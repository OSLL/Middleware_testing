//
// Created by egor on 20.07.2020.
//

#ifndef OPENDDS_DEVGUIDE_MESSENGER_PING_PONG_H
#define OPENDDS_DEVGUIDE_MESSENGER_PING_PONG_H

#include <Subscriber.h>
#include <Publisher.h>
#include "../../../interface/ping_pong_interface.hpp"

class TestPingPongNode: public TestMiddlewarePingPong<Messenger::Message>{
public:
    TestPingPongNode( int argc, ACE_TCHAR *argv[],
                      std::string &topic1, std::string &topic2, int msgCount, int prior, int cpu_index,
                      std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst );

    bool receive() override;

    short get_id(Messenger::Message &msg) override;

    unsigned long get_timestamp(Messenger::Message &msg) override;

    void publish(short id, unsigned size) override;

private:
    DDS::DomainParticipant_var _participant;
    DDS::DomainParticipantFactory_var _dpf;

    Messenger::MessageDataWriter_var _message_writer;
    DDS::DataWriter_var _writer;
    DDS::WaitSet_var _ws;
    DDS::Duration_t _timeout{};
    Messenger::Message _message;

    Messenger::MessageSeq _messages;
    DDS::SampleInfoSeq _info;
    DDS::Subscriber_var _subscriber;

    DDS::StatusCondition_var _condition;
    DDS::DataReader_var _reader;
    Messenger::MessageDataReader_var _reader_i;
    DDS::DataReaderListener_var _listener;
    Messenger::MessageTypeSupport_var _ts;

    int _id = 0;


};

#endif //OPENDDS_DEVGUIDE_MESSENGER_PING_PONG_H
