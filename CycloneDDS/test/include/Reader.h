//
// Created by egor on 20.03.20.
//

#ifndef CYCLONE_TEST_READER_H
#define CYCLONE_TEST_READER_H

#include <dds/dds.h>
#include <QoS.h>
#include <Partisipant.h>
#include <Topic.h>
#include <iostream>

#include <TypeData.h>

#define MAX_SAMPLES 1

template <class MsgType>
class Reader {

public:
    Reader() = default;

    Reader(Participant participant, Topic topic){
        QoS qos;
        _topic = topic;
        _participant = participant;
        _reader = dds_create_reader (_participant._participant, _topic._topic, qos._qos, nullptr);
        if (_reader < 0)
            DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-_reader));

        _samples[0] = Messenger_Message__alloc ();
    };

    bool receive(){
            _res_code = dds_take (_reader, _samples, _infos, MAX_SAMPLES, MAX_SAMPLES);
            if (_res_code < 0) {
                DDS_FATAL("dds_read: %s\n", dds_strretcode(-_res_code));
            }

            if ((_res_code > 0) && (_infos[0].valid_data)) {
                _msg = (MsgType*) _samples[0];
                return true;
            }

            return false;
    }

    void deleteQoS(){
        _qos.deleteQos();
    };

    void *_samples[MAX_SAMPLES];
    dds_sample_info_t _infos[MAX_SAMPLES];
    dds_return_t _res_code = 0;
    dds_entity_t _reader = 0;
    Topic _topic;
    Participant _participant;
    MsgType* _msg;
    QoS _qos;

};


#endif //CYCLONE_TEST_READER_H
