#ifndef CYCLONE_TEST_PING_PONG_H
#define CYCLONE_TEST_PING_PONG_H
#include <string>
#include <chrono>

#include <dds/dds.h>
#include <TypeData.h>

#include <ping_pong_interface.hpp>

#define MAX_SAMPLES 1

using namespace std::chrono;

template <class MsgType>
class PingPong: public TestMiddlewarePingPong<MsgType> {

public:
    PingPong(std::string &topic1, std::string &topic2, int msgCount, int prior,
             int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst) :
            TestMiddlewarePingPong<MsgType>(topic1, topic2, msgCount, prior, cpu_index, filename, topic_priority,
                                            msInterval, msgSize, isFirst)
            {};

    PingPong(std::string &topic1, std::string &topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSizeMin, int msgSizeMax,
            int step, int before_step, bool isFirst):
            TestMiddlewarePingPong<MsgType>(topic1, topic2, msgCount, prior, cpu_index, filename, topic_priority,
                                            msInterval, msgSizeMin, msgSizeMax, step, before_step, isFirst)
            {};


    void create(dds_topic_descriptor topic_descriptor);

    void publish(short id, unsigned size) override;

    short get_id(MsgType& msg) override;

    unsigned long get_timestamp(MsgType& msg) override;

    bool receive() override;

    ~PingPong()
    {
        _res_code = dds_delete (_participant);
        if (_res_code != DDS_RETCODE_OK)
            DDS_FATAL("dds_delete: %s\n", dds_strretcode(-_res_code));
    };

private:

    void write_to_topic(short id, std::string &data);

    MsgType *_rec_msg;
    MsgType _sent_msg;
    int32_t _res_code;
    dds_entity_t _participant;

    dds_entity_t _writer_entity ;
    dds_entity_t _topic_entity1;

    dds_entity_t _reader_entity;
    dds_entity_t _topic_entity2;

    void *_samples[MAX_SAMPLES+1];
    dds_sample_info_t _infos[MAX_SAMPLES+1];

};

#endif //CYCLONE_TEST_PING_PONG_H
