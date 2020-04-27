//
// Created by egor on 20.03.20.
//

#ifndef CYCLONE_TEST_QOS_H
#define CYCLONE_TEST_QOS_H

#include <dds/dds.h>

class QoS {

public:

    QoS(){
        _qos = dds_create_qos ();
        setReliable(10);
        setTransientLocal(10);
    };

    void setReliable(int sec){
        dds_qset_reliability (_qos, DDS_RELIABILITY_RELIABLE, DDS_SECS (10));
        //dds_qset_durability_service(_qos, DDS_DURABILITY_TRANSIENT_LOCAL,)
    };

    void setTransientLocal(int sec){
        dds_qset_reliability (_qos, DDS_RELIABILITY_RELIABLE, DDS_SECS (10));
        //dds_qset_durability_service(_qos, DDS_DURABILITY_TRANSIENT_LOCAL,)
    };

    void deleteQos(){
        dds_delete_qos(_qos);
    };

    dds_qos* _qos = nullptr;
};


#endif //CYCLONE_TEST_QOS_H
