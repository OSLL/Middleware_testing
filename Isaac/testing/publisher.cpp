#include "publisher.hpp"
#include "control.hpp"
#define WATERMARK 50


void Publisher::start()  {
    if(std::stoi(get_tick_period()) > 0){
        tickPeriodically();
    }else{
        tickBlocking();
    }
    pub = std::make_unique<PublisherComponent>("", get_msg_count(),
        get_prior(), get_cpu_index(), get_min_msg_size(),
        get_max_msg_size(), get_step(), 0, get_msgs_before_step(),
        get_filename(), 0);
    cur = 0;
    msg_count = pub->get_msg_count();
    min_size = pub->get_min_msg_size();
    max_size = pub->get_max_msg_size();
    size = min_size;
    step = pub->get_step();
    before_step = pub->get_before_step();
    is_ping_pong = get_ping_pong();
    std::this_thread::sleep_for(std::chrono::seconds(4));
    if(is_ping_pong){
        sub = (Subscriber*)node()->findComponentByName("sub");
        if(!sub) throw test_exception("No component storage", TEST_ERROR);
    }
}

void Publisher::tick()  {
    if(is_ping_pong && cur - sub->last_id > WATERMARK) return;
    if(cur % (before_step - 1) == 0 && size <= max_size)
        size += step;
    std::string str(size, 'a');
    unsigned long start, end;
    start = std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count();
    auto msg = tx_send().initProto();
    msg.setId(cur);
    msg.setData(str);
    msg.setTimestamp(start);
    tx_send().publish();
    end = std::chrono::duration_cast<std::chrono::
    nanoseconds>(std::chrono::high_resolution_clock::
                 now().time_since_epoch()).count();
    if(is_ping_pong && sub->pub) sub->pub->write_proc_msg(cur, end - start);
    else pub->write_proc_msg(cur, end - start);
    if(++cur >= msg_count) reportSuccess();
}

void Publisher::stop()  {
    if(!is_ping_pong){
        pub->to_Json();
        Control* control = node()->app()->getNodeComponentOrNull<Control>("control");
        if(!control) throw test_exception("No component control", TEST_ERROR);
        control->node_complete();
    }
    LOG_INFO("End Publisher");
}
