//
// Created by egor on 15.04.2020.
//

#ifndef OPENDDS_DEVGUIDE_MESSENGER_PUBLISHER_H
#define OPENDDS_DEVGUIDE_MESSENGER_PUBLISHER_H

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

#include <argparse/argparse.hpp>

#include <pub_interface.hpp>

class Publisher: public TestMiddlewarePub{

    std::string a = "mock-string";

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

    Publisher() :
            TestMiddlewarePub(
                    a, 0, 0, 0, 0, 0,
                    0, 0, 0, a, 0)
    {};

    void createPublisher(int argc, ACE_TCHAR *argv[]);

    unsigned long publish(short id, unsigned size, char alpha);

    unsigned long publish(short id, unsigned size) override;

    void cleanUp();

private:
    std::string _topic;
    int _msInterval;
    int _msgCount;
    int _priority; //not stated
    int _cpu_index; //not stated
    int _byteSizeMin;
    int _byteSizeMax;
    int _step;
    int _msg_count_befor_step;

    DDS::DomainParticipant_var _participant;
    Messenger::MessageDataWriter_var _message_writer;
    DDS::DataWriter_var _writer;
    DDS::DomainParticipantFactory_var _dpf;
    DDS::WaitSet_var _ws;
    DDS::Duration_t _timeout{};
    Messenger::Message _message;

};


#endif //OPENDDS_DEVGUIDE_MESSENGER_PUBLISHER_H
