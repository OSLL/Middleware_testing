#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include <unistd.h>
#include "gen/TestData_DCPS.hpp"
#include "test_interface.hpp"

class TestPublisher: public TestMiddlewarePub{
    dds::domain::DomainParticipant _dp;
    dds::topic::Topic <TestDataType> _topic;
    dds::pub::Publisher _publisher;
    dds::pub::DataWriter <TestDataType> _dw;
public:
    TestPublisher(std::string topic,  int msgCount=0, int prior = -1, int cpu_index = -1,
                  int min_msg_size=50, int max_msg_size=64000, int step=0, int interval = 0, int msgs_before_step = 100):
                  TestMiddlewarePub(topic, msgCount, prior, cpu_index, min_msg_size, max_msg_size, step, interval, msgs_before_step),
                  _dp(org::opensplice::domain::default_id()),
                  _topic(_dp,topic),
                  _publisher(_dp),
                  _dw(_publisher, _topic)
                  {
        setQoS("file://QoS.xml");
    }

    void setQoS(std::string filename) override {
        dds::core::QosProvider provider(filename, "TestProfile");
    }

    void publish(short id, unsigned size) override {
        unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        std::string data('a', size);
        TestDataType msg(id, cur_time, std::vector<char>(data.begin(), data.end()));
        _dw.write(msg);
    }
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
    std::string topic;
    int m_count = 5000;
    int priority = -1;
    int cpu_index = -1;
    int min_msg_size = 0;
    int max_msg_size = 64000;
    int step = 0;
    int msgs_before_step = 100;
    int interval = 0;

    file >> args;
    std::cout<<args;
    file.close();

    if(args["topic"] != nullptr){
        topic = args["topic"];
    }
    if(args["m_count"] != nullptr){
        m_count = args["m_count"];
    }
    if(args["min_msg_size"] != nullptr){
        min_msg_size = args["min_msg_size"];
    }
    if(args["max_msg_size"] != nullptr){
        max_msg_size = args["max_msg_size"];
    }
    if(args["step"] != nullptr){
        step = args["step"];
    }
    if(args["msgs_before_step"] != nullptr){
        msgs_before_step = args["msgs_before_step"];
    }
    if(args["priority"] != nullptr){
        priority = args["priority"];
    }
    if(args["cpu_index"] != nullptr){
        cpu_index = args["cpu_index"];
    }
    if(args["interval"] != nullptr){
        interval = args["interval"];
    }
    try {
        TestPublisher publisher(topic, m_count, priority, cpu_index, min_msg_size, max_msg_size, step, interval, msgs_before_step);
        publisher.StartTest();
    }
    catch (...){
        std::cout<< "Error!\n";
        return -1;
    }
}