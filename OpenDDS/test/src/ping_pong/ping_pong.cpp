#include "ping_pong.h"


TestPingPongNode::TestPingPongNode( int argc, ACE_TCHAR *argv[],
        std::string &topic1, std::string &topic2, int msgCount, int prior, int cpu_index,
        std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst ):
        TestMiddlewarePingPong<Messenger::Message>(
                topic1, topic2, msgCount, prior, cpu_index,filename,
                topic_priority, msInterval, msgSize, isFirst ){
    try {
        // Initialize Domain_ParticipantFactory
        _dpf = TheParticipantFactoryWithArgs(argc, argv);

        // Create Domain_Participant
        std::hash<std::string> hash_fn;
        auto domainId = (unsigned int) hash_fn(_topic_name1 + _topic_name2) % 100000;
        if(!isFirst)
            std::swap(_topic_name1, _topic_name2);
        _participant = _dpf->create_participant(domainId,
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

        DDS::Topic_var ptopic = _participant->create_topic(_topic_name1.c_str(),
                                                          type_name.in(),
                                                          TOPIC_QOS_DEFAULT,
                                                          DDS::TopicListener::_nil(),
                                                          OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        if (CORBA::is_nil(ptopic.in())) {
            ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -") ACE_TEXT(" create_topic failed!\n")));
            return;
        }

        DDS::Topic_var stopic = _participant->create_topic(_topic_name2.c_str(),
                                                           type_name.in(),
                                                           TOPIC_QOS_DEFAULT,
                                                           DDS::TopicListener::_nil(),
                                                           OpenDDS::DCPS::DEFAULT_STATUS_MASK);

        if (CORBA::is_nil(stopic.in())) {
            ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -") ACE_TEXT(" create_topic failed!\n")));
            return;
        }

        init_subscriber(stopic);
        init_publisher(ptopic);
    }
    catch (const CORBA::Exception& e) {
        e._tao_print_exception("Exception caught in main():");
        return;
    }
}

bool TestPingPongNode::receive() {
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
            //std::cout<<"rec msg " << get_id(_messages[0]) << "from "<< _topic_name2 <<std::endl;
            write_received_msg(_messages[0]);
            _read_msg_time[get_id(_messages[0])] = proc_time;
            _reader_i->return_loan(_messages, _info);
            return true;
        }
    }
    _reader_i->return_loan(_messages, _info);
    return false;
}

short TestPingPongNode::get_id(Messenger::Message &msg) {
    return msg.id;
}

unsigned long TestPingPongNode::get_timestamp(Messenger::Message &msg) {
    return msg.timestamp;
}

void TestPingPongNode::publish(short id, unsigned size, char alpha) {
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count();


    //std::cout << "message " << id << " sent to "<< _topic_name1 << std::endl;

    // Write samples
    _message.id = id;

    _message.timestamp = std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count();

    std::string data(size, alpha);
    _message.payload = data.c_str();

    DDS::ReturnCode_t error = _message_writer->write(_message, DDS::HANDLE_NIL);
    _write_msg_time[id] =std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count() - cur_time;
    if (error != DDS::RETCODE_OK) {
        ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -")
                          ACE_TEXT(" write returned %d!\n"), error));
    }


}

void TestPingPongNode::publish(short id, unsigned size) {
    if (_isFirst)
        publish(id, size, 'a');
    else
        publish(id, size, 'b');
};

void TestPingPongNode::init_publisher(DDS::Topic_var &topic) {
    // Create Publisher
    DDS::Publisher_var publisher =
            _participant->create_publisher(PUBLISHER_QOS_DEFAULT,
                                           DDS::PublisherListener::_nil(),
                                           OpenDDS::DCPS::DEFAULT_STATUS_MASK);

    if (CORBA::is_nil(publisher.in())) {
        ACE_ERROR((LM_ERROR, ACE_TEXT("ERROR: %N:%l: main() -") ACE_TEXT(" create_publisher failed!\n")));
        return;
    }

    DDS::DataWriterQos writer_qos;
    publisher->get_default_datawriter_qos(writer_qos);
    writer_qos.reliability.kind = DDS::RELIABLE_RELIABILITY_QOS;
    writer_qos.durability.kind = DDS::TRANSIENT_LOCAL_DURABILITY_QOS;

    // Create DataWriter
    _writer = publisher->create_datawriter(topic.in(),
                                           writer_qos,
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
}

void TestPingPongNode::init_subscriber(DDS::Topic_var &topic) {
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
}

void TestPingPongNode::cleanUp() {
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
}
