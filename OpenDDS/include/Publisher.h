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

#include "../../interface/pub_interface.hpp"


class Publisher: public TestMiddlewarePub{

    std::string a = "mock-string";

public:
    Publisher(std::string &topic, int msg_count, int prior, int cpu_index, int min_msg_size, int max_msg_size,
              int step, int interval, int msgs_before_step, std::string& file_name, int topic_priority) :
            TestMiddlewarePub(
                    topic, msg_count, prior, cpu_index, min_msg_size, max_msg_size,
                    step, interval, msgs_before_step, file_name, topic_priority)
            {};

    void createPublisher(int argc, ACE_TCHAR *argv[]);

    unsigned long publish(short id, unsigned size, char alpha);

    unsigned long publish(short id, unsigned size) override;

    ~Publisher(){
        cleanUp();
    }

    void cleanUp();

private:
    DDS::DomainParticipant_var _participant;
    Messenger::MessageDataWriter_var _message_writer;
    DDS::DataWriter_var _writer;
    DDS::DomainParticipantFactory_var _dpf;
    Messenger::Message _message;

};


#endif //OPENDDS_DEVGUIDE_MESSENGER_PUBLISHER_H
