//
// Created by egor on 27.04.2020.
//

//
// Created by egor on 02.04.20.
//

#include <Subscriber.h>


template <class MsgType>
void Subscriber<MsgType>::create(dds_topic_descriptor topic_descriptor) {

    auto qos = QoS();
    _participant_entity = Participant(0, qos);
    _topic_entity = Topic(_participant_entity, _topic, topic_descriptor);
    _reader_entity = Reader<MsgType>(_participant_entity, _topic_entity);

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

    if (_reader_entity.receive()) {
        unsigned long proc_time = std::chrono::duration_cast<std::chrono::
        nanoseconds>(std::chrono::high_resolution_clock::
        now().time_since_epoch()).count()
        - start_timestamp;

        this->write_received_msg(*_reader_entity._msg, proc_time);
        return true;
    }

    return false;
}


template <class MsgType>
void Subscriber<MsgType>::cleanUp() {
    std::cout << "Participant.close(): " << _participant_entity.close() << std::endl;
}



template class Subscriber<Messenger_Message>;