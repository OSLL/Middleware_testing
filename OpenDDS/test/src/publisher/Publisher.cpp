//
// Created by egor on 02.04.20.
//

#include <Publisher.h>

void Publisher::createPublisher(int argc, ACE_TCHAR *argv[]) {

    try {
        // Initialize Domain_ParticipantFactory
        _dpf = TheParticipantFactoryWithArgs(argc, argv);

        // Create Domain_Participant
        _participant = _dpf->create_participant(42,
                PARTICIPANT_QOS_DEFAULT,
                DDS::DomainParticipantListener::_nil(),
                OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        if (CORBA::is_nil(_participant.in())) {
            ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -") ACE_TEXT(" create_participant failed!\n")));
            return;
        }

        // Register TypeSupport (Messenger::Message)
        Messenger::MessageTypeSupport_var ts = new Messenger::MessageTypeSupportImpl();

        if (ts->register_type(_participant.in(), "") != DDS::RETCODE_OK) {
            ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -") ACE_TEXT(" register_type failed!\n")));
            return;
        }

        // Create Topic (Movie Discussion List)
        CORBA::String_var type_name = ts->get_type_name();
        DDS::Topic_var topic = _participant->create_topic(_topic.c_str(),
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

    } catch (const CORBA::Exception& e) {
        e._tao_print_exception("Exception caught in main():");
        return;
    }

};

unsigned long Publisher::publish(short id, unsigned size) {

    unsigned long cur_time = std::chrono::duration_cast<std::chrono::
            nanoseconds>(std::chrono::high_resolution_clock::
            now().time_since_epoch()).count();

    // Write samples
    _message.id = id;

    _message.timestamp = std::chrono::duration_cast<std::chrono::
            nanoseconds>(std::chrono::high_resolution_clock::
            now().time_since_epoch()).count();

    std::string data(size, 'a');
    _message.payload = data.c_str();

    DDS::ReturnCode_t error = _message_writer->write(_message, DDS::HANDLE_NIL);

    if (error != DDS::RETCODE_OK) {
        ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -")
                          ACE_TEXT(" write returned %d!\n"), error));
        return 0;
    }

    return std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
    now().time_since_epoch()).count()
    - cur_time;
};

void Publisher::cleanUp(){
    // Clean-up!
    _participant->delete_contained_entities();
    _dpf->delete_participant(_participant.in());

    TheServiceParticipant->shutdown();
};

