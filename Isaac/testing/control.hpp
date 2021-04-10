#pragma once
#include "engine/alice/alice_codelet.hpp"
#include "packages/testing/message.capnp.h"

class Control: public isaac::alice::Codelet {
private:
    int nodes_completed, all_nodes;
public:
    void start() override;

    void tick() override;

    void stop() override;

    void node_complete();

    ISAAC_PROTO_RX(Message, none);
    
    ISAAC_PARAM(int, node_count, 0);
};

ISAAC_ALICE_REGISTER_CODELET(Control);
