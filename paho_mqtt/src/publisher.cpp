#include "../include/publisher.hpp"


TestPublisher::TestPublisher(std::string topic,  int msgCount, int prior, int cpu_index,
                             int min_msg_size, int max_msg_size, int step, int interval,
                             int msgs_before_step, std::string &filename,
                             int topic_priority):
        TestMiddlewarePub(topic, msgCount, prior, cpu_index, min_msg_size, max_msg_size, step, interval,
                          msgs_before_step, filename, topic_priority),
        _client(ADDRESS, filename, mqtt::create_options(MQTTVERSION_5))
                          {
    _client.connect()->wait();
    _topic = new mqtt::topic(_client, topic, QOS);
                          }

unsigned long TestPublisher::publish(short id, unsigned int size) {
    std::string data(size, 'a');
    unsigned long proc_time = 0;
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    mqtt::token_ptr tok;
    data = std::to_string(id) + "time:" + std::to_string(cur_time) + "data:" + data;
    tok = _topic->publish(data);

    tok->wait();
    // std:: cout << "Sent msg " << id << std::endl;
    proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time;
    return proc_time;
}

TestPublisher::~TestPublisher() {
    delete _topic;
    _client.disconnect()->wait();
}
