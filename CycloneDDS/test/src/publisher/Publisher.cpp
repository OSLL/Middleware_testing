//
// Created by egor on 27.04.2020.
//

#include <Publisher.h>

template <class MsgType>
void Publisher<MsgType>::create(dds_topic_descriptor topic_descriptor) {

    auto qos = QoS();
    _participant_entity = Participant(0, qos);
    _topic_entity = Topic(_participant_entity, _topic, topic_descriptor);
    _writer_entity = Writer<MsgType>(_participant_entity, _topic_entity);
}

template <class MsgType>
unsigned long Publisher<MsgType>::publish(short id, unsigned size){

    unsigned long cur_time = std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count();

    _msg.timestamp = std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count();

    _msg.id = id;

    std::string data(size, 'a');
    _msg.payload = const_cast<char*>(data.c_str());

    _res_code = _writer_entity.write(_msg);

    if (_res_code != DDS_RETCODE_OK) {
        std::cout <<  dds_strretcode(-_res_code) << std::endl;
        return 0;
    }

    return std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
    now().time_since_epoch()).count()
    - cur_time;

};


template <class MsgType>
void Publisher<MsgType>::cleanUp() {
    std::cout << "Participant.close(): " << _participant_entity.close() << std::endl;
}

template class Publisher<Messenger_Message>;