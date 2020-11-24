#ifndef CYCLONE_TEST_PUBLISHER_H
#define CYCLONE_TEST_PUBLISHER_H

#include <string>
#include <chrono>

#include <dds/dds.h>
#include <TypeData.h>

#include <pub_interface.hpp>


using namespace std::chrono;

template <class MsgType>
class Publisher: public TestMiddlewarePub {

public:
    Publisher(std::string &topic, int msg_count, int prior, int cpu_index, int min_msg_size, int max_msg_size,
              int step, int interval, int msgs_before_step, std::string& file_name, int topic_priority) :
            TestMiddlewarePub(
                    topic, msg_count, prior, cpu_index, min_msg_size, max_msg_size,
                    step, interval, msgs_before_step, file_name, topic_priority)
    {};

    void create(dds_topic_descriptor topic_descriptor);

    unsigned long publish(short id, unsigned size) override;

    ~Publisher()
    {
        _res_code = dds_delete (_participant);
        if (_res_code != DDS_RETCODE_OK)
            DDS_FATAL("dds_delete: %s\n", dds_strretcode(-_res_code));
    };

private:
    MsgType _msg;
    int32_t _res_code;
    dds_entity_t _writer_entity ;
    dds_entity_t _topic_entity;

    dds_entity_t _participant;

};


#endif //CYCLONE_TEST_PUBLISHER_H
