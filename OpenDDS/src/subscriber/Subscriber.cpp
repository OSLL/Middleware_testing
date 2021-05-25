//
// Created by egor on 02.04.20.
//

#include "Subscriber.h"
#include <time.h>
#include <stdlib.h>

void Subscriber::createSubscriber(int argc, ACE_TCHAR *argv[]) {
    try {
        // Initialize DomainParticipantFactory
        _dpf = TheParticipantFactoryWithArgs(argc, argv);

        // Create DomainParticipant
        std::hash<std::string> hash_fn;
        auto domainId = (unsigned int) hash_fn(_topic_name) % 100000;
        _participant = _dpf->create_participant(domainId,
                                        PARTICIPANT_QOS_DEFAULT,
                                        DDS::DomainParticipantListener::_nil(),
                                        OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        if (CORBA::is_nil(_participant.in())) {
            ACE_ERROR((LM_ERROR,
                    ACE_TEXT("ERROR: %N:%l: main() -")
                    ACE_TEXT(" create_participant failed!\n")));
        }

        // Register Type (Messenger::Message)
        auto _ts = new Messenger::MessageTypeSupportImpl();

        if (_ts->register_type(_participant.in(), "") != DDS::RETCODE_OK) {
            ACE_ERROR((LM_ERROR,
                    ACE_TEXT("ERROR: %N:%l: main() -")
                    ACE_TEXT(" register_type failed!\n")));
        }

        // Create Topic (Movie Discussion List)
        CORBA::String_var type_name = _ts->get_type_name();
        DDS::Topic_var topic =
                _participant->create_topic(_topic_name.c_str(),
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

        DDS::DataReaderQos reader_qos;
        _subscriber->get_default_datareader_qos(reader_qos);
        reader_qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;

        // Create DataReader
        _listener = new DataReaderListenerImpl;

        _reader = _subscriber->create_datareader(topic.in(),
                                                       reader_qos,
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


short Subscriber::get_id(Messenger::Message& msg) {
    return msg.id;
};

unsigned long Subscriber::get_timestamp(Messenger::Message& msg) {
    return msg.timestamp;
};


bool Subscriber::receive() {

    auto start_timestamp = std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count();

    DDS::ReturnCode_t error = _reader_i->take(_messages,
                                              _info,
                                              1,
                                              DDS::ANY_SAMPLE_STATE,
                                              DDS::ANY_VIEW_STATE,
                                              DDS::ANY_INSTANCE_STATE);


    auto proc_time = std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count()
                              - start_timestamp;

    if (error == DDS::RETCODE_OK) {
        if (_info[0].valid_data) {
            // std::cout<<"rec msg " << get_id(_messages[0]) <<std::endl;
            write_received_msg(_messages[0], proc_time);
            _reader_i->return_loan(_messages, _info);
            return true;
        }
    }
    _reader_i->return_loan(_messages, _info);
    return false;
};


void Subscriber::cleanUp(){
    // Clean-up!
    std::cout << "CleanUp started!\n";
    try {
        if (!CORBA::is_nil (_participant.in ())) {
            _participant->delete_contained_entities();
        }
        if (!CORBA::is_nil (_dpf.in ())) {
            _dpf->delete_participant(_participant.in ());
        }
    } catch (CORBA::Exception& e) {
        std::cout << "Exception caught in cleanup." << std::endl << e << std::endl;
        ACE_OS::exit(1);
    }
    TheServiceParticipant->shutdown();
};
