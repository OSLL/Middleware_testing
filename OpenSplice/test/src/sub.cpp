#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include <unistd.h>
#include "gen/TestData_DCPS.hpp"
#include "../../../interface/sub_interface.hpp"

#include <cmath>
#define TIMEOUT_NS 2*pow(10, 10)

class TestSubscriber: public TestMiddlewareSub{
public:
    TestSubscriber(std::vector<std::string> &topics, std::vector<std::string> filenames, int msgCount=0, int prior=-1, int cpu_index=-1):
            TestMiddlewareSub(topics, msgCount, prior, cpu_index, filenames),
            _dp(org::opensplice::domain::default_id()),
            _provider("file://QoS.xml", "TestProfile"),
            _subscriber(_dp)
            {
                for(auto it = _topic_names.begin(); it != _topic_names.end(); it++){
                    dds::topic::Topic<TestDataType> topic(_dp, *it, _provider.topic_qos());
                    _topics.push_back(topic);
                    _drs.push_back(dds::sub::DataReader<TestDataType>(_subscriber, topic, _provider.datareader_qos()));
                }
            }


    int receive(int topic_id) override {
        int i = 0;
        unsigned long start_timer = 0;
        while(i < _msgCount){
            auto samples = _drs[0].read();
            for(auto j=samples.begin();  j != samples.end(); ++j){
                if(j->info().state().sample_state() == dds::sub::status::SampleState::not_read()){
                    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                    start_timer = cur_time;
                    msgs[topic_id][i].first = j->data().id();
                    msgs[topic_id][i].second = j->data().sent_time();
                    rec_time[topic_id][i] = cur_time;
                    std::cout<<j->data().id()<<std::endl;
                    i++;
                }
                else{
                    unsigned long end_timer = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                    if(start_timer != 0 && end_timer-start_timer > TIMEOUT_NS)
                        throw;
                }

            }
        }
        return i;
    }
private:
    dds::domain::DomainParticipant _dp;
    dds::core::QosProvider _provider;
    std::vector<dds::topic::Topic<TestDataType>> _topics;
    dds::sub::Subscriber _subscriber;
    std::vector<dds::sub::DataReader <TestDataType>> _drs;
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
    nlohmann::json topics;
    std::vector<std::string> res_filenames;
    std::string res_filename = "res.json";
    int m_count = 5000;
    int priority = -1;
    int cpu_index = -1;

    file >> args;
    file.close();

    if(args["topics"] != nullptr){
        topics = args["topics"];
    }
    if(args["res_filenames"] != nullptr) {
        for (auto res_filename : args["res_filenames"])
            res_filenames.push_back(res_filename);
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
        std::vector<std::string> topic_names(topics.begin(), topics.end());
        TestSubscriber subscriber(topic_names, res_filenames, m_count, priority, cpu_index);
        subscriber.StartTest();
    }
    catch (...){
        std::cout<< "Error!\n";
        return -1;
    }
}