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

    PingPong(std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSizeMin, int msgSizeMax,
            int step, int before_step, bool isFirst):
            TestMiddlewarePingPong<MsgType>(topic1, topic2, msgCount, prior, cpu_index, filename, topic_priority,
                                            msInterval, msgSizeMin, msgSizeMax, step, before_step, isFirst)
            {};


    void create(dds_topic_descriptor topic_descriptor);

    unsigned long publish(short id, unsigned size) override;

    void cleanUp() override ;

    short get_id(MsgType& msg) override;

    unsigned long get_timestamp(MsgType& msg) override;

    bool receive() override;

private:
    MsgType _msg;
    int32_t _res_code;
    dds_entity_t _writer_entity ;
    dds_entity_t _topic_entity1;

    dds_entity_t _participant;
    dds_entity_t _topic_entity2;
    dds_entity_t _reader_entity;

    void *_samples[MAX_SAMPLES];
    dds_sample_info_t _infos[MAX_SAMPLES];

};
#endif //CYCLONE_TEST_PING_PONG_H
