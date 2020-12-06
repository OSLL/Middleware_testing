#ifndef CYCLONE_TEST_SUBSCRIBER_H
#define CYCLONE_TEST_SUBSCRIBER_H

#include <string>
#include <chrono>

#include <sub_interface.hpp>
#include <dds/dds.h>
#include <TypeData.h>

#define MAX_SAMPLES 1

using namespace std::chrono;

template <class MsgType>
class Subscriber: public TestMiddlewareSub<MsgType>{

public:
    Subscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_priority) :
            TestMiddlewareSub<MsgType>(topic, msgCount, prior, cpu_index, filename, topic_priority)
    {};

    void create(dds_topic_descriptor topic_descriptor);

    short get_id(MsgType& msg) override;

    unsigned long get_timestamp(MsgType& msg) override;

    bool receive() override;

    ~Subscriber()
    {
        _res_code = dds_delete (_participant);
        if (_res_code != DDS_RETCODE_OK)
            DDS_FATAL("dds_delete: %s\n", dds_strretcode(-_res_code));

        //std::cout << "Participant.close(): " << dds_strretcode(-_res_code) << std::endl;
    };


private:
    MsgType* _msg;
    int32_t _res_code;
    dds_entity_t _topic_entity;
    dds_entity_t _reader_entity;
    dds_entity_t _participant;
    void *_samples[MAX_SAMPLES];
    dds_sample_info_t _infos[MAX_SAMPLES];
};


#endif //CYCLONE_TEST_SUBSCRIBER_H
