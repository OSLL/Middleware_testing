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
#include "dds/DataReaderListenerImpl.h"

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
            //_sub_interface(TestMiddlewareSub<Messenger::Message>(topic, msgCount, prior, cpu_index, filename, topic_priority))
            {};

    void createSubscriber(int argc, ACE_TCHAR *argv[]) {
        try {
            // Initialize DomainParticipantFactory
            _dpf = TheParticipantFactoryWithArgs(argc, argv);

            // Create DomainParticipant
            _participant =
                    _dpf->create_participant(42,
                                            PARTICIPANT_QOS_DEFAULT,
                                            DDS::DomainParticipantListener::_nil(),
                                            OpenDDS::DCPS::DEFAULT_STATUS_MASK);

            if (CORBA::is_nil(_participant.in())) {
                ACE_ERROR((LM_ERROR,
                        ACE_TEXT("ERROR: %N:%l: main() -")
                                         ACE_TEXT(" create_participant failed!\n")));
            }

            // Register Type (Messenger::Message)
            _ts = new Messenger::MessageTypeSupportImpl();

            if (_ts->register_type(_participant.in(), "") != DDS::RETCODE_OK) {
                ACE_ERROR((LM_ERROR,
                        ACE_TEXT("ERROR: %N:%l: main() -")
                                         ACE_TEXT(" register_type failed!\n")));
            }

            // Create Topic (Movie Discussion List)
            CORBA::String_var type_name = _ts->get_type_name();
            DDS::Topic_var topic =
                    _participant->create_topic("test_topic",
                                              type_name.in(),
                                              TOPIC_QOS_DEFAULT,
                                              DDS::TopicListener::_nil(),
                                              OpenDDS::DCPS::DEFAULT_STATUS_MASK);

            if (CORBA::is_nil(topic.in())) {
                ACE_ERROR((LM_ERROR,
                        ACE_TEXT("ERROR: %N:%l: main() -")
                                         ACE_TEXT(" create_topic failed!\n")));
            }

            // Create Subscriber
            _subscriber = _participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT,
                                                   DDS::SubscriberListener::_nil(),
                                                   OpenDDS::DCPS::DEFAULT_STATUS_MASK);

            if (CORBA::is_nil(_subscriber.in())) {
                ACE_ERROR((LM_ERROR,
                        ACE_TEXT("ERROR: %N:%l: main() -")
                                         ACE_TEXT(" create_subscriber failed!\n")));
            }

            // Create DataReader
            _listener = new DataReaderListenerImpl;

            _reader = _subscriber->create_datareader(topic.in(),
                                                  DATAREADER_QOS_DEFAULT,
                                                  _listener.in(),
                                                  OpenDDS::DCPS::DEFAULT_STATUS_MASK);

            if (CORBA::is_nil(_reader.in())) {
                ACE_ERROR((LM_ERROR,
                        ACE_TEXT("ERROR: %N:%l: main() -")
                                         ACE_TEXT(" create_datareader failed!\n")));
            }

            _reader_i = Messenger::MessageDataReader::_narrow(_reader);

            if (CORBA::is_nil(_reader_i.in())) {
                ACE_ERROR((LM_ERROR,
                        ACE_TEXT("ERROR: %N:%l: main() -")
                                         ACE_TEXT(" _narrow failed!\n")));
            }



        } catch (const CORBA::Exception& e) {
            e._tao_print_exception("Exception caught in main():");
        }

    }


    bool receive(Messenger::Message& msg) {
        Messenger::MessageSeq messages;
        DDS::SampleInfoSeq info;

        DDS::ReturnCode_t error = _reader_i->take(messages,
                                                 info,
                                                 1,
                                                 DDS::ANY_SAMPLE_STATE,
                                                 DDS::ANY_VIEW_STATE,
                                                 DDS::ANY_INSTANCE_STATE);

        if (error == DDS::RETCODE_OK) {
            if (info[0].valid_data) {
                TestMiddlewareSub<Messenger::Message>::receive(messages[0]);
                _reader_i->return_loan(messages, info);
                return true;
            } else
                return false;

        } else {
            ACE_ERROR((LM_ERROR,
                    ACE_TEXT("ERROR: %N:%l: on_data_available() -")
                              ACE_TEXT(" take failed!\n")));
        }
        _reader_i->return_loan(messages, info);
        return false;
    };

    void setQoS(std::string filename) {

    };

    void cleanUp(){
        // Clean-up!
        _participant->delete_contained_entities();
        _dpf->delete_participant(_participant.in());

        TheServiceParticipant->shutdown();
    };

protected:
    std::string _topic_name;
    int _msgCount;
    int _priority; //def not stated
    int _cpu_index; //def not stated
    std::string _filename;

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