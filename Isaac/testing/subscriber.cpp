#include "subscriber.hpp"
#include "control.hpp"

void Subscriber::start() {
    msg_count = get_msg_count() - 1;
    last_id = 0;
    is_ping_pong = get_ping_pong();
    if(is_ping_pong){
        ping_pong = std::make_unique<PingPongComponent>("", "",
            msg_count + 1, get_prior(), get_cpu_index(), get_filename(), 0, 0, 0, 0, 0, 0, true);
        pub = std::make_unique<PublisherComponent>("", get_msg_count(),
            get_prior(), get_cpu_index(), 0,
            0, 0, 0, 0,
            get_filename(), 0);
        if(!pub || !ping_pong) throw test_exception("No memory", TEST_ERROR);
    }else{
        sub = std::make_unique<SubscriberComponent>("", msg_count + 1,
            get_prior(), get_cpu_index(), get_filename(), 0);
    }
    tickOnMessage(rx_receive());
}

void Subscriber::tick() {
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
    if(is_ping_pong){
        ping_pong->write_received_msg(msg);
        ping_pong->write_read_time(msg.id, end - start);
        last_id = msg.id;
    }else sub->write_received_msg(msg, end - start);
    if(msg.id >= msg_count) reportSuccess();
}

void Subscriber::stop(){
    if(is_ping_pong){
        auto times = pub->get_proc_times();
        for(size_t i = 0; i < times.size(); i++)
            ping_pong->write_proc_time(i, times[i]);
        ping_pong->to_json();
    } else sub->to_json();
    LOG_INFO("End Subscriber");
    Control* control = node()->app()->getNodeComponentOrNull<Control>("control");
    if(!control) throw test_exception("No component control", TEST_ERROR);
    control->node_complete();
}

int Subscriber::get_last_id(){return last_id;}

void Subscriber::write_proc_time(int id, unsigned long time){
    if(!is_ping_pong) return;
    ping_pong->write_proc_time(id, time);
}


