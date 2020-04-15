#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include <unistd.h>
#include "gen/TestData_DCPS.hpp"
#include "../../../interface/pub_interface.hpp"

class TestPublisher: public TestMiddlewarePub{
public:
    TestPublisher(std::string topic,  int msgCount, int prior, int cpu_index,
            int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step, std::string &filename,
            int topic_priority):
            TestMiddlewarePub(topic, msgCount, prior, cpu_index, min_msg_size, max_msg_size, step, interval,
                    msgs_before_step, filename, topic_priority),
            _dp(org::opensplice::domain::default_id()),
            _provider("file://QoS.xml", "TestProfile"),
            _topic(_dp, _topic_name, _provider.topic_qos()),
            _publisher(_dp),
            _dw(_publisher, _topic, _provider.datawriter_qos())
            {}


    unsigned long publish(short id, unsigned size) override {
        unsigned long proc_time = 0;
        std::string data(size, 'a');
        unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        TestDataType msg(id, cur_time, std::vector<char>(data.begin(), data.end()));
        cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        _dw.write(msg);
        proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time;
        return proc_time;
    }

private:
    dds::domain::DomainParticipant _dp;
    dds::core::QosProvider _provider;
    dds::topic::Topic <TestDataType> _topic;
    dds::pub::Publisher _publisher;
    dds::pub::DataWriter <TestDataType> _dw;
};


int main(int argc, char **argv) {
    if(argc < 2) {
        std::cout << "Config file is not set!\n";
        return 0;
    }
    nlohmann::json args;
    std::ifstream file(argv[1]);
    if(!file.is_open()) {
        std::cout << "Cannot open file " << argv[1] << std::endl;
        return 0;
    }
    file >> args;
    file.close();

    std::string topic = args["topic"];
    std::string filename = args["res_filenames"][0];
    int m_count = args["m_count"];
    int priority = args["priority"][0];
    int cpu_index = args["cpu_index"][0];
    int min_msg_size = args["min_msg_size"];
    int max_msg_size = args["max_msg_size"];
    int step = args["step"];
    int msgs_before_step = args["msgs_before_step"];
    int interval = args["interval"];
    int topic_prior = args["topic_priority"];

    try {
        TestPublisher publisher(topic, m_count, priority, cpu_index, min_msg_size, max_msg_size, step, interval,
                msgs_before_step, filename, topic_prior);
        publisher.StartTest();
    }
    catch (test_exception& e){
        std::cout<< e.what() << std::endl;
        return -e.get_ret_code();
    }
    catch (std::exception& e){
        std::cout<< e.what()<< std::endl;
        return -MIDDLEWARE_ERROR;
    }
}