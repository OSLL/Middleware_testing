#include "control.hpp"

void Control::start(){
    nodes_completed = 0;
    all_nodes = get_node_count();
    if(all_nodes <= 0) reportSuccess();
    tickOnMessage(rx_none());
}

void Control::tick(){}

void Control::stop(){
    LOG_INFO("Stop Application");
}

void Control::node_complete(){
    if(++nodes_completed >= all_nodes) reportSuccess();
}
