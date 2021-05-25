#ifndef OPENDDS_DEVGUIDE_MESSENGER_SUBSCRIBER_H
#define OPENDDS_DEVGUIDE_MESSENGER_SUBSCRIBER_H

#include <ace/Log_Msg.h>

#include <dds/DdsDcpsInfrastructureC.h>
#include <dds/DdsDcpsPublicationC.h>
#include <dds/DCPS/Marked_Default_Qos.h>
#include <dds/DCPS/Service_Participant.h>
#include <dds/DCPS/WaitSet.h>
#include "dds/DCPS/StaticIncludes.h"
#include <dds/DCPS/WaitSet.h>
#include <MessengerTypeSupportImpl.h>
#include <DataReaderListenerImpl.h>



#include "../../interface/sub_interface.hpp"

#include <argparse/argparse.hpp>


class Subscriber: public TestMiddlewareSub<Messenger::Message>{

public:
    Subscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_priority) :
            TestMiddlewareSub<Messenger::Message>(topic, msgCount, prior, cpu_index, filename, topic_priority)
    {};

    void createSubscriber(int argc, ACE_TCHAR *argv[]);

    short get_id(Messenger::Message& msg) override;

    unsigned long get_timestamp(Messenger::Message& msg) override;

    bool receive() override;

    ~Subscriber(){
        cleanUp();
    }

    void cleanUp();

protected:
    Messenger::MessageSeq _messages;
    DDS::SampleInfoSeq _info;
    DDS::DomainParticipant_var _participant;
    DDS::Subscriber_var _subscriber;
    DDS::DomainParticipantFactory_var _dpf;
    DDS::DataReader_var _reader;
    Messenger::MessageDataReader_var _reader_i;
    DDS::DataReaderListener_var _listener;
};

#endif //OPENDDS_DEVGUIDE_MESSENGER_SUBSCRIBER_H
