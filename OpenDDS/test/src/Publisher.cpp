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

#include <pub_interface.hpp>

#include <argparse/argparse.hpp>



class Publisher: public TestMiddlewarePub{

public:
    Publisher(std::string &topic,  int msgCount, int prior, int cpu_index,
              int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step) :
            _topic(topic),
            _msInterval(interval),
            _msgCount(msgCount),
            _priority(prior),
            _cpu_index(cpu_index),
            _byteSizeMin(min_msg_size),
            _byteSizeMax(max_msg_size),
            _step(step),
            _msg_count_befor_step(msgs_before_step),
            TestMiddlewarePub(topic, msgCount, prior, cpu_index, min_msg_size, max_msg_size, step, interval, msgs_before_step){};

    void createPublisher(int argc, ACE_TCHAR *argv[]) {
        try {
            // Initialize Domain_ParticipantFactory
            _dpf =
                    TheParticipantFactoryWithArgs(argc, argv);

            // Create Domain_Participant
            _participant =
                    _dpf->create_participant(42,
                                            PARTICIPANT_QOS_DEFAULT,
                                            DDS::DomainParticipantListener::_nil(),
                                            OpenDDS::DCPS::DEFAULT_STATUS_MASK);

            if (CORBA::is_nil(_participant.in())) {
                ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -") ACE_TEXT(" create_participant failed!\n")));
                return;
            }

            // Register TypeSupport (Messenger::Message)
            Messenger::MessageTypeSupport_var ts =
                    new Messenger::MessageTypeSupportImpl();

            if (ts->register_type(_participant.in(), "") != DDS::RETCODE_OK) {
                ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -") ACE_TEXT(" register_type failed!\n")));
                return;
            }

            // Create Topic (Movie Discussion List)
            CORBA::String_var type_name = ts->get_type_name();
            DDS::Topic_var topic =
                    _participant->create_topic(_topic.c_str(),
                                              type_name.in(),
                                              TOPIC_QOS_DEFAULT,
                                              DDS::TopicListener::_nil(),
                                              OpenDDS::DCPS::DEFAULT_STATUS_MASK);

            if (CORBA::is_nil(topic.in())) {
                ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -") ACE_TEXT(" create_topic failed!\n")));
                return;
            }

            // Create Publisher
            DDS::Publisher_var publisher =
                    _participant->create_publisher(PUBLISHER_QOS_DEFAULT,
                                                  DDS::PublisherListener::_nil(),
                                                  OpenDDS::DCPS::DEFAULT_STATUS_MASK);

            if (CORBA::is_nil(publisher.in())) {
                ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -") ACE_TEXT(" create_publisher failed!\n")));
                return;
            }

            // Create DataWriter
            _writer = publisher->create_datawriter(topic.in(),
                                                 DATAWRITER_QOS_DEFAULT,
                                                 DDS::DataWriterListener::_nil(),
                                                 OpenDDS::DCPS::DEFAULT_STATUS_MASK);

            if (CORBA::is_nil(_writer.in())) {
                ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -") ACE_TEXT(" create_datawriter failed!\n")));
                return;
            }

            _message_writer = Messenger::MessageDataWriter::_narrow(_writer.in());

            if (CORBA::is_nil(_message_writer.in())) {
                ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -") ACE_TEXT(" _narrow failed!\n")));
                return;
            }

            // Block until Subscriber is available
            _condition = _writer->get_statuscondition();
            _condition->set_enabled_statuses(DDS::PUBLICATION_MATCHED_STATUS);

            _ws = new DDS::WaitSet;
            _ws->attach_condition(_condition);

            DDS::ConditionSeq conditions;
            DDS::PublicationMatchedStatus matches = { 0, 0, 0, 0, 0 };
            _timeout = { 30, 0 };

            do {
                if (_ws->wait(conditions, _timeout) != DDS::RETCODE_OK) {
                    ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -") ACE_TEXT(" wait failed!\n")));
                    return;
                }

                if (_writer->get_publication_matched_status(matches) != ::DDS::RETCODE_OK) {
                    ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -")
                                      ACE_TEXT(" get_publication_matched_status failed!\n")));
                    return;
                }

            } while (matches.current_count < 1);



        } catch (const CORBA::Exception& e) {
            e._tao_print_exception("Exception caught in main():");
            return;
        }

    }


    void publish(short id, unsigned size) {


        _ws->detach_condition(_condition);

        // Write samples
        Messenger::Message message;
        message.subject_id = id;

        message.from       = CORBA::string_dup("Comic Book Guy");
        message.subject    = CORBA::string_dup("Review");
        message.text       = CORBA::string_dup("Worst. Movie. Ever.");
        message.count      = 0;
        DDS::ReturnCode_t error = _message_writer->write(message, DDS::HANDLE_NIL);

        if (error != DDS::RETCODE_OK) {
            ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -")
                              ACE_TEXT(" write returned %d!\n"), error));
            return;
        }

        // Wait for samples to be acknowledged
        if (_message_writer->wait_for_acknowledgments(_timeout) != DDS::RETCODE_OK) {
            ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -")
                              ACE_TEXT(" wait_for_acknowledgments failed!\n")));
            return;
        }

    };

    void setQoS(std::string filename) {

    };

    void cleanUp(){
        // Clean-up!
        _participant->delete_contained_entities();
        _dpf->delete_participant(_participant.in());

        TheServiceParticipant->shutdown();
    };

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
    DDS::StatusCondition_var _condition;
    DDS::Duration_t _timeout;

};