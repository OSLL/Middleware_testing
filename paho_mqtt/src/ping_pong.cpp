#include "../include/ping_pong.hpp"


TestPingPongNode::TestPingPongNode(std::string &topic1, std::string &topic2, int msgCount, int prior, int cpu_index,
                                   std::string &filename, int topic_priority, int msInterval, int msgSize,
                                   bool isFirst):
        TestMiddlewarePingPong<std::string>(topic1, topic2, msgCount, prior, cpu_index, filename, topic_priority,
                                             msInterval, msgSize, isFirst),
        _client_sub(ADDRESS, filename+"1", mqtt::create_options(MQTTVERSION_5)),
        _client_pub(ADDRESS, filename+"2", mqtt::create_options(MQTTVERSION_5))
{
    init();
}

TestPingPongNode::TestPingPongNode(std::string &topic1, std::string topic2, int msgCount, int prior, int cpu_index,
                                   std::string &filename, int topic_priority, int msInterval, int msgSizeMin,
                                   int msgSizeMax, int step, int before_step, bool isFirst) : TestMiddlewarePingPong(
        topic1, topic2, msgCount, prior, cpu_index, filename, topic_priority, msInterval, msgSizeMin, msgSizeMax, step,
        before_step, isFirst),
        _client_sub(ADDRESS, filename+"1", mqtt::create_options(MQTTVERSION_5)),
        _client_pub(ADDRESS, filename+"2", mqtt::create_options(MQTTVERSION_5))
{
    init();
}

bool TestPingPongNode::receive() {
    auto start_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    if (_last_arrived_id >= _msgCount - 1 || (_isFirst && _last_sent_id == _last_arrived_id))
        return false;
    auto msg = _client_sub.consume_message();
    unsigned long proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - start_timestamp;
    if (msg) {
        std::string payload = msg->to_string();
        write_received_msg(payload);
        _read_msg_time[get_id(payload)] = proc_time;
        _last_arrived_id = get_id(payload);
        // std::cout << "Rec msg: " << _last_arrived_id << std::endl;
        return true;
    }
    return false;
}

short TestPingPongNode::get_id(std::string &msg) {
    return std::stoi(msg.substr(0, sizeof("time:")));
}

unsigned long TestPingPongNode::get_timestamp(std::string &msg) {
    auto start_pos = msg.find("time:") + sizeof("time:") - 1;
    auto len = msg.find("data") - start_pos;
    return std::stoll(msg.substr(start_pos, len));
}

void TestPingPongNode::publish(short id, unsigned int size) {
    std::string data(size, 'a');
    unsigned long proc_time = 0;
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    mqtt::token_ptr tok;
    data = std::to_string(id) + "time:" + std::to_string(cur_time) + "data:" + data;
    tok = _topic->publish(data);

    tok->wait();
    proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time;
    _write_msg_time[id]=proc_time;
    _last_sent_id = id;
    // std:: cout << "Sent msg " << id << std::endl;
}

void TestPingPongNode::init() {
    auto connOpts = mqtt::connect_options_builder()
            .mqtt_version(MQTTVERSION_5)
            .automatic_reconnect(std::chrono::milliseconds (200), std::chrono::seconds(1))
            .clean_session(false)
            .finalize();

    mqtt::connect_response rsp = _client_sub.connect(connOpts);
    _client_pub.connect()->wait();
    std::string t1 = _topic_name1, t2 = _topic_name2;
    if (! _isFirst){
        t1 = _topic_name2;
        t2 = _topic_name1;
    }
    _topic = new mqtt::topic(_client_pub, t1, QOS);
    if (!rsp.is_session_present()) {
        _client_sub.subscribe(t2, QOS);
    }
    else {
        std::cout << "Session already present. Skipping subscribe." << std::endl;
    }
}
