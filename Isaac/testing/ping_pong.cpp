#include "ping_pong.hpp"
#include "control.hpp"

void PingPong::start(){
    msg_count = get_msg_count();
    min_size = get_min_msg_size();
    max_size = get_max_msg_size();
    step = get_step();
    before_step = get_msgs_before_step();
    size = min_size;
    first = get_first();
    ping_pong = std::make_unique<PingPongComponent>("", "", msg_count,
        get_prior(), get_cpu_index(), get_filename(), 0, 0, min_size, max_size,
        step, before_step, first);
    tickOnMessage(rx_receive());
    LOG_INFO(("Start PingPong " + node()->name()).c_str());
    if(first){
        unsigned long start = std::chrono::duration_cast<std::chrono::
        nanoseconds>(std::chrono::high_resolution_clock::
                     now().time_since_epoch()).count();
        std::string str(size, 'a');
        auto msg = tx_send().initProto();
        msg.setId(0);
        msg.setData(str);
        msg.setTimestamp(start);
        tx_send().publish();
        unsigned long end = std::chrono::duration_cast<std::chrono::
        nanoseconds>(std::chrono::high_resolution_clock::
                     now().time_since_epoch()).count();
        ping_pong->write_proc_time(0, end - start);
    }
}

void PingPong::tick(){
    unsigned long start, end;
    start = std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count();
    auto message = rx_receive().getProto();
    Msg msg;
    std::string data = message.getData();
    msg.id = message.getId();
    msg.timestamp = message.getTimestamp();
    end = std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count();
    ping_pong->write_received_msg(msg);
    ping_pong->write_read_time(msg.id, end - start);
    if(first) msg.id++;
    if(msg.id >= msg_count && first) reportSuccess();
    else{
        if(msg.id % (before_step - 1) == 0 && size <= max_size)
            size += step;
        start = std::chrono::duration_cast<std::chrono::
        nanoseconds>(std::chrono::high_resolution_clock::
                     now().time_since_epoch()).count();
        std::string str(size, 'a');
        auto new_msg = tx_send().initProto();
        new_msg.setId(msg.id);
        new_msg.setData(str);
        new_msg.setTimestamp(start);
        tx_send().publish();
        end = std::chrono::duration_cast<std::chrono::
        nanoseconds>(std::chrono::high_resolution_clock::
                     now().time_since_epoch()).count();
        ping_pong->write_proc_time(msg.id, end - start);
        if(msg.id >= msg_count - 1) reportSuccess();
    }
}

void PingPong::stop(){
    ping_pong->to_json();
    LOG_INFO("End PingPong");
    Control* control = node()->app()->getNodeComponentOrNull<Control>("control");
    if(!control) throw test_exception("No component control", TEST_ERROR);
    control->node_complete();
}
