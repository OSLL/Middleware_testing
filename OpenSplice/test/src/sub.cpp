#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include <unistd.h>
#include "gen/TestData_DCPS.hpp"
#include "test_interface.hpp"


class TestSubscriber: public TestMiddlewareSub{
public:
    TestSubscriber(std::string topic,  int msgCount=0, int prior=-1, int cpu_index=-1, std::string filename = "res.json"):
            TestMiddlewareSub(topic, msgCount, prior, cpu_index, filename),
            _dp(org::opensplice::domain::default_id()),
            _topic(_dp,topic),
            _subscriber(_dp),
            _dr(_subscriber, _topic)
            {
                setQoS("file://QoS.xml");
            }


    void setQoS(std::string filename) override {
        dds::core::QosProvider provider(filename, "TestProfile");
    }

    int receive() override {
        int i = 0;
        while(i < _msgCount){
            auto samples = _dr.read();
            for(auto j=samples.begin();  j != samples.end(); ++j){
                if(j->info().state().sample_state() == dds::sub::status::SampleState::not_read()){
                    msgs[i].first = j->data().id();
                    msgs[i].second = j->data().sent_time();
                    i++;
                }
            }
        }
        return i;
    }

private:
    dds::domain::DomainParticipant _dp;
    dds::topic::Topic <TestDataType> _topic;
    dds::sub::Subscriber _subscriber;
    dds::sub::DataReader <TestDataType> _dr;
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
    std::string res_filename = "res.json";
    int m_count = 5000;
    int priority = -1;
    int cpu_index = -1;

    file >> args;
    std::cout<<args;
    file.close();

    if(args["topic"] != nullptr){
        topic = args["topic"];
    }
    if(args["res_filename"] != nullptr){
        res_filename = args["res_filename"];
    }
    if(args["m_count"] != nullptr){
        m_count = args["m_count"];
    }
    if(args["priority"] != nullptr){
        priority = args["priority"];
    }
    if(args["cpu_index"] != nullptr){
        cpu_index = args["cpu_index"];
    }
    try {
        TestSubscriber subscriber(topic, m_count, priority, cpu_index, res_filename);
        subscriber.StartTest();
    }
    catch (...){
        std::cout<< "Error!\n";
        return -1;
    }
}