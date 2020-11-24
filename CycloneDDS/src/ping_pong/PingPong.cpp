#include <PingPong.h>

template <class MsgType>
void PingPong<MsgType>::create(dds_topic_descriptor topic_descriptor) {

    /* Create a Participant. */
    _participant = dds_create_participant (DDS_DOMAIN_DEFAULT, NULL, NULL);
    if (_participant < 0)
        DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-_participant));

    /* Create a Topic. */
    _topic_entity1 = dds_create_topic (
            _participant, &topic_descriptor, "test_topic", NULL, NULL);
    if (_topic_entity1 < 0)
        DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-_topic_entity1));

    /* Create a reliable Reader. */
    auto _qos = dds_create_qos ();
    dds_qset_reliability (_qos, DDS_RELIABILITY_RELIABLE, DDS_SECS (10));

    /* Create a Writer. */
    _writer_entity = dds_create_writer (_participant, _topic_entity1, NULL, NULL);
    if (_writer_entity < 0)
        DDS_FATAL("dds_create_writer: %s\n", dds_strretcode(-_writer_entity));

    std::cout << "=== [Publisher]  Waiting for a reader to be discovered ..." << std::endl;

    auto rc = dds_set_status_mask(_writer_entity, DDS_PUBLICATION_MATCHED_STATUS);
    if (rc != DDS_RETCODE_OK)
        DDS_FATAL("dds_set_status_mask: %s\n", dds_strretcode(-rc));

    uint32_t status = 0;

    while(!(status & DDS_PUBLICATION_MATCHED_STATUS))
    {
        rc = dds_get_status_changes( _writer_entity, &status);
        if (rc != DDS_RETCODE_OK)
            DDS_FATAL("dds_get_status_changes: %s\n", dds_strretcode(-rc));

        /* Polling sleep. */
        dds_sleepfor (DDS_MSECS (20));
    }

}

template <class MsgType>
unsigned long PingPong<MsgType>::publish(short id, unsigned size){

    unsigned long cur_time = std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count();

    _msg.timestamp = std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count();

    _msg.id = id;


    std::string data(size, 'a');
    _msg.payload = const_cast<char*>(data.c_str());

    _res_code =  dds_write(_writer_entity, &_msg);

    if (_res_code != DDS_RETCODE_OK) {
        std::cout <<  dds_strretcode(-_res_code) << std::endl;
        return 0;
    }

    //std::cout << "sent: " << id << std::endl;

    return std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count()
           - cur_time;

};

template <class MsgType>
short PingPong<MsgType>::get_id(MsgType& msg) {
    return msg.id;
};

template <class MsgType>
unsigned long PingPong<MsgType>::get_timestamp(MsgType& msg) {
    return msg.timestamp;
};

template <class MsgType>
bool PingPong<MsgType>::receive(){
    auto start_timestamp = std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count();


    _res_code = dds_take (_reader_entity, _samples, _infos, MAX_SAMPLES, MAX_SAMPLES);

    if (_res_code < 0) {
        DDS_FATAL("dds_read: %s\n", dds_strretcode(-_res_code));
    }

    if ((_res_code > 0) && (_infos[0].valid_data)) {
        unsigned long proc_time = std::chrono::duration_cast<std::chrono::
        nanoseconds>(std::chrono::high_resolution_clock::
                     now().time_since_epoch()).count()
                                  - start_timestamp;

        _msg = (MsgType*) _samples[0];

        this->write_received_msg(*_msg, proc_time);

        std::cout << _msg->id << std::endl;
        return true;
    }

    return false;
}


template <class MsgType>
void PingPong<MsgType>::cleanUp() {
    _res_code = dds_delete (_participant);
    if (_res_code != DDS_RETCODE_OK)
        DDS_FATAL("dds_delete: %s\n", dds_strretcode(-_res_code));

    //std::cout << "Participant.close(): " << dds_strretcode(-_res_code) << std::endl;
}




template <class MsgType>
void PingPong<MsgType>::cleanUp() {
}


template class PingPong<Messenger_Message>;

