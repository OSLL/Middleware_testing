#pragma once
#include "engine/alice/alice.hpp"
#include "packages/testing/message.capnp.h"
#include "sub_comp.hpp"
#include "pub_comp.hpp"
#include "ping_pong_comp.hpp"
#include <memory>

class Subscriber: public isaac::alice::Codelet {
private:
    std::unique_ptr<SubscriberComponent> sub;

    std::unique_ptr<PingPongComponent> ping_pong;

    int msg_count;

    bool is_ping_pong;
public:
    std::unique_ptr<PublisherComponent> pub;

    int last_id;

    void start() override;

    void tick() override;

    void stop() override;

    int get_last_id();

    void write_proc_time(int id, unsigned long time);

    ISAAC_PROTO_RX(Message, receive);

    ISAAC_PARAM(int, msg_count, 0);

    ISAAC_PARAM(int, prior, -1);

    ISAAC_PARAM(int, cpu_index, -1);

    ISAAC_PARAM(std::string, filename, "sub_json.hpp");

    ISAAC_PARAM(bool, ping_pong, false);
};

ISAAC_ALICE_REGISTER_CODELET(Subscriber);
