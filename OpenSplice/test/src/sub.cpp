#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "gen/TestData_DCPS.hpp"
#include "../../../interface/sub_interface.hpp"


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
                    _drs.back().default_filter_state(dds::sub::status::DataState::new_data());
                }
            }


    int receive(int topic_id) override {
        auto samples = _drs[topic_id].read();
        if(samples.length() > 0){
            unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            auto id = samples.begin()->data().id();
            msgs[topic_id][id].first = id;
            msgs[topic_id][id].second = samples.begin()->data().sent_time();
            rec_time[topic_id][id] = cur_time;
            std::cout<<samples.begin()->data().id()<<std::endl;
        }
        else
            return 0;
        return 1;
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
    catch (test_exception& e){
        std::cout<< e.what() << std::endl;
        return -e.get_ret_code();
    }
    catch (std::exception& e){
        std::cout<< e.what()<< std::endl;
        return -MIDDLEWARE_ERROR;
    }
}