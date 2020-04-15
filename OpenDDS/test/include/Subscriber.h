//
// Created by egor on 15.04.2020.
//

#ifndef OPENDDS_DEVGUIDE_MESSENGER_SUBSCRIBER_H
#define OPENDDS_DEVGUIDE_MESSENGER_SUBSCRIBER_H

//
// Created by nata on 02.04.20.
//


#include <ace/Log_Msg.h>

#include <dds/DdsDcpsInfrastructureC.h>
#include <dds/DdsDcpsPublicationC.h>
#include <dds/DCPS/Marked_Default_Qos.h>
#include <dds/DCPS/Service_Participant.h>
#include <dds/DCPS/WaitSet.h>

#include "dds/DCPS/StaticIncludes.h"

#include "MessengerTypeSupportImpl.h"
#include "../src/dds/DataReaderListenerImpl.h"

#include <sub_interface.hpp>

#include <argparse/argparse.hpp>


class Subscriber: public TestMiddlewareSub<Messenger::Message>{

public:
    Subscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_priority) :
            _topic_name(topic),
            _msgCount(msgCount),
            _priority(prior),
            _cpu_index(cpu_index),
            _filename(filename),
            TestMiddlewareSub<Messenger::Message>(topic, msgCount, prior, cpu_index, filename, topic_priority)
    {};

    void createSubscriber(int argc, ACE_TCHAR *argv[]);

    short get_id(Messenger::Message& msg) override;

    unsigned long get_timestamp(Messenger::Message& msg) override;

    bool receive() override;

    void cleanUp();

protected:
    std::string _topic_name;
    int _msgCount;
    int _priority; //def not stated
    int _cpu_index; //def not stated
    std::string _filename;

    Messenger::MessageSeq _messages;
    DDS::SampleInfoSeq _info;
    DDS::DomainParticipant_var _participant;
    DDS::Subscriber_var _subscriber;
    DDS::DomainParticipantFactory_var _dpf;
    DDS::WaitSet_var _ws;
    DDS::StatusCondition_var _condition;
    DDS::Duration_t _timeout;
    DDS::DataReader_var _reader;
    Messenger::MessageDataReader_var _reader_i;
    DDS::DataReaderListener_var _listener;
    Messenger::MessageTypeSupport_var _ts;
    //TestMiddlewareSub<Messenger::Message> _sub_interface;

};

#endif //OPENDDS_DEVGUIDE_MESSENGER_SUBSCRIBER_H
