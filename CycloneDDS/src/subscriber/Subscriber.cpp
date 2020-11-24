#include <Subscriber.h>


template <class MsgType>
void Subscriber<MsgType>::create(dds_topic_descriptor topic_descriptor) {

    /* Create a Participant. */
    _participant = dds_create_participant (DDS_DOMAIN_DEFAULT, NULL, NULL);
    if (_participant < 0)
        DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-_participant));

    /* Create a Topic. */
    _topic_entity = dds_create_topic (
            _participant, &topic_descriptor, TestMiddlewareSub<MsgType>::_topic_name.c_str(), NULL, NULL);
    if (_topic_entity < 0)
        DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-_topic_entity));

    /* Create a reliable Reader. */
    auto _qos = dds_create_qos ();
    dds_qset_reliability (_qos, DDS_RELIABILITY_RELIABLE, DDS_SECS (10));
    _reader_entity = dds_create_reader (_participant, _topic_entity, _qos, NULL);
    if (_reader_entity < 0)
        DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-_reader_entity));

    std::cout << "=== [Subscriber] Waiting for a sample ..." << std::endl;

    /* Initialize sample buffer, by pointing the void pointer within
     * the buffer array to a valid sample memory location. */
    _samples[0] = Messenger_Message__alloc ();

};

template <class MsgType>
short Subscriber<MsgType>::get_id(MsgType& msg) {
    return msg.id;
};

template <class MsgType>
unsigned long Subscriber<MsgType>::get_timestamp(MsgType& msg) {
    return msg.timestamp;
};

template <class MsgType>
bool Subscriber<MsgType>::receive(){
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

        //std::cout << _msg->id << std::endl;
        return true;
    }

    return false;
}

template class Subscriber<Messenger_Message>;