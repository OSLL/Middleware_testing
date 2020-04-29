//
// Created by egor on 20.03.20.
//

#ifndef CYCLONE_TEST_DOMAIN_H
#define CYCLONE_TEST_DOMAIN_H


#include <QoS.h>
#include <dds/dds.h>
#include <cstdlib>
#include <iostream>


class Participant {

public:

    dds_entity_t _participant = 0;

    Participant() = default;

    explicit Participant(int domain_id, QoS qos){
        assert(domain_id >= 0 && domain_id <= 230);

        _participant = dds_create_participant (domain_id, qos._qos, nullptr);
        std::cout << "participant create: " << dds_strretcode(-_participant) << std::endl;
        if (_participant < 0)
            DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-_participant));
    };

    dds_return_t close(){
        auto _res_code = dds_delete (_participant);
        if (_res_code != DDS_RETCODE_OK)
            DDS_FATAL("dds_delete: %s\n", dds_strretcode(-_res_code));

        return _res_code;
    }

};


#endif //CYCLONE_TEST_DOMAIN_H
