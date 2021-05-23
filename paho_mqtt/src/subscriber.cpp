#include "../include/subscriber.hpp"


bool TestSubscriber::receive() {
    auto start_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    if (_last_id >= _msgCount - 1)
        return false;
    auto msg = _client.consume_message();
    unsigned long proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - start_timestamp;
    if (msg) {
        std::string payload = msg->to_string();
        write_received_msg(payload, proc_time);
        _last_id = get_id(payload);
        // std::cout << "Rec msg: " << _last_id << std::endl;
        return true;
    }
    return false;
}

short TestSubscriber::get_id(std::string &msg) {
    return std::stoi(msg.substr(0, sizeof("time:")));
}

unsigned long TestSubscriber::get_timestamp(std::string &msg) {
    auto start_pos = msg.find("time:") + sizeof("time:") - 1;
    auto len = msg.find("data") - start_pos;
    return std::stoll(msg.substr(start_pos, len));
}

TestSubscriber::TestSubscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename,
                               int topic_priority) : TestMiddlewareSub(topic, msgCount, prior, cpu_index, filename,
                                                                       topic_priority),
                                                     _client(ADDRESS, filename, mqtt::create_options(MQTTVERSION_5))
                                                     {

    auto connOpts = mqtt::connect_options_builder()
            .mqtt_version(MQTTVERSION_5)
            .automatic_reconnect(std::chrono::seconds(2), std::chrono::seconds(30))
            .clean_session(false)
            .finalize();

    mqtt::connect_response rsp = _client.connect(connOpts);

    if (!rsp.is_session_present()) {
        _client.subscribe(_topic_name, QOS);
    }
    else {
        std::cout << "Session already present. Skipping subscribe." << std::endl;
    }
}
