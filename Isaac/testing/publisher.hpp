#pragma once
#include <memory>
#include "engine/alice/alice_codelet.hpp"
#include "pub_comp.hpp"
#include "packages/testing/message.capnp.h"
#include "subscriber.hpp"

class Publisher: public isaac::alice::Codelet {
private:
    std::unique_ptr<PublisherComponent> pub;

    int size;

    int cur, msg_count, min_size, max_size, step, before_step;

    bool is_ping_pong;

    Subscriber* sub;
public:
    void start() override;

    void tick() override;

    void stop() override;

    ISAAC_PROTO_TX(Message, send);

    ISAAC_PARAM(int, msg_count, 0);

    ISAAC_PARAM(int, prior, -1);

    ISAAC_PARAM(int, cpu_index, -1);

    ISAAC_PARAM(int, min_msg_size, 0);

    ISAAC_PARAM(int, max_msg_size, 0);

    ISAAC_PARAM(int, step, 0);

    ISAAC_PARAM(int, msgs_before_step, 1);

    ISAAC_PARAM(std::string, filename, "pub.json");

    ISAAC_PARAM(bool, ping_pong, false);
};

ISAAC_ALICE_REGISTER_CODELET(Publisher);
