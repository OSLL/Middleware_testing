#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "gen/TestData_DCPS.hpp"
#include "../../../interface/sub_interface.hpp"


class TestSubscriber: public TestMiddlewareSub{
public:
    TestSubscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_priority):
            TestMiddlewareSub(topic, msgCount, prior, cpu_index, filename, topic_priority),
            _dp(org::opensplice::domain::default_id()),
            _provider("file://QoS.xml", "TestProfile"),
            _topic(_dp, topic, _provider.topic_qos()),
            _subscriber(_dp),
            _dr(_subscriber, _topic, _provider.datareader_qos())
            {
                _dr.default_filter_state(dds::sub::status::DataState::new_data());
            }


    int receive() override {
        unsigned long start_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        auto samples = _dr.read();
        if(samples.length() > 0){
            auto id = samples.begin()->data().id();
            _read_msg_time[id] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - start_time;
            unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            msgs[id].first = id;
            msgs[id].second = samples.begin()->data().sent_time();
            rec_time[id] = cur_time;
            //std::cout<<samples.begin()->data().id()<<std::endl;
        }
        else
            return 0;
        return 1;
    }
private:
    dds::domain::DomainParticipant _dp;
    dds::core::QosProvider _provider;
    dds::topic::Topic<TestDataType> _topic;
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
    file >> args;
    file.close();
    std::string topic = args["topic"];
    std::string filename = args["res_filenames"][1];
    int m_count = args["m_count"];
    int priority = args["priority"][1];
    int cpu_index = args["cpu_index"][1];
    int topic_prior = args["topic_priority"];
    try {
        TestSubscriber subscriber(topic, m_count, priority, cpu_index, filename, topic_prior);
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