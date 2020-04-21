#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "gen/TestData_DCPS.hpp"
#include "../../../interface/sub_interface.hpp"


class TestSubscriber: public TestMiddlewareSub<TestDataType>{
public:
    TestSubscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_priority):
            TestMiddlewareSub<TestDataType>(topic, msgCount, prior, cpu_index, filename, topic_priority),
            _dp(org::opensplice::domain::default_id()),
            _provider("file://QoS.xml", "TestProfile"),
            _topic(_dp, topic, _provider.topic_qos()),
            _subscriber(_dp),
            _dr(_subscriber, _topic, _provider.datareader_qos())
            {
                _dr.default_filter_state(dds::sub::status::DataState::new_data());
            }


    bool receive() override {
        auto start_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        auto samples = _dr.read();
        unsigned long proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - start_timestamp;
        if(samples.length() > 0){
            auto msg = samples.begin()->data();
            write_received_msg(msg, proc_time);
            //std::cout<< msg.id()<<std::endl;
        }
        else
            return false;
        return true;
    }

    short get_id(TestDataType &msg) override {
        return msg.id();
    }

    unsigned long get_timestamp(TestDataType &msg) override {
        return msg.timestamp();
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