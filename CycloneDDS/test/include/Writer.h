//
// Created by egor on 20.03.20.
//

#ifndef CYCLONE_TEST_WRITER_H
#define CYCLONE_TEST_WRITER_H

#include <dds/dds.h>

template <class MsgType>
class Writer {

public:
    Writer() = default;

    Writer(Participant participant, Topic topic) {
        _writer = dds_create_writer (participant._participant, topic._topic, NULL, NULL);

        std::cout << "writer: " << dds_strretcode(-_writer) << std::endl;

        if (_writer < 0)
            DDS_FATAL("dds_create_writer: %s\n", dds_strretcode(-_writer));

        _res_code = dds_set_status_mask(_writer, DDS_PUBLICATION_MATCHED_STATUS);

        std::cout << "writer mask: " << dds_strretcode(-_writer) << std::endl;


        if (_res_code != DDS_RETCODE_OK)
            DDS_FATAL("dds_set_status_mask: %s\n", dds_strretcode(-_res_code));
    }

    dds_return_t write(MsgType &_msg){
        return dds_write(_writer, &_msg);
    }

private:
    dds_return_t _res_code = 0;
    dds_entity_t _writer = 0;

};


#endif //CYCLONE_TEST_WRITER_H
