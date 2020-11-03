//
// Created by egor on 20.07.2020.
//

#ifndef OPENDDS_DEVGUIDE_MESSENGER_PING_PONG_H
#define OPENDDS_DEVGUIDE_MESSENGER_PING_PONG_H

#include <Subscriber.h>
#include <Publisher.h>
#include "../../../interface/ping_pong_interface.hpp"

class TestPingPongNode: public TestMiddlewarePingPong<Messenger::Message>{
public:
    TestPingPongNode( int argc, ACE_TCHAR *argv[],
                      std::string &topic1, std::string &topic2, int msgCount, int prior, int cpu_index,
                      std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst );

    bool receive() override;

    short get_id(Messenger::Message &msg) override;

    unsigned long get_timestamp(Messenger::Message &msg) override;

    void publish(short id, unsigned size) override;

private:
    Subscriber sub;
    Publisher pub;
};

#endif //OPENDDS_DEVGUIDE_MESSENGER_PING_PONG_H
