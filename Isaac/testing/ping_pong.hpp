#pragma once
#include <memory>
#include "engine/alice/alice_codelet.hpp"
#include "ping_pong_comp.hpp"
#include "packages/testing/message.capnp.h"

class PingPong: public isaac::alice::Codelet {
private:
    std::unique_ptr<PingPongComponent> ping_pong;

    bool first;

    int msg_count, min_size, max_size, step, before_step, size;
public:
    void start() override;

    void tick() override;

    void stop() override;

    ISAAC_PROTO_TX(Message, send);

    ISAAC_PROTO_RX(Message, receive);

    ISAAC_PARAM(int, msg_count, 0);

    ISAAC_PARAM(int, prior, -1);

    ISAAC_PARAM(int, cpu_index, -1);

    ISAAC_PARAM(int, min_msg_size, 0);

    ISAAC_PARAM(int, max_msg_size, 0);

    ISAAC_PARAM(int, step, 0);

    ISAAC_PARAM(int, msgs_before_step, 1);

    ISAAC_PARAM(std::string, filename, "pub.json");

    ISAAC_PARAM(bool, first, true);
};

ISAAC_ALICE_REGISTER_CODELET(PingPong);
