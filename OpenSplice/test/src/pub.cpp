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

    void publish(std::string data) override {
        TestDataType msg(std::vector<char>(data.begin(), data.end()));
        _dw.write(msg);
    }
};


int main(int argc, char **argv) {
    try {
        TestPublisher publisher("/topic", 100);
        publisher.StartTest();
    }
    catch (...){
        std::cout<< "Error!\n";
        return -1;
    }
}