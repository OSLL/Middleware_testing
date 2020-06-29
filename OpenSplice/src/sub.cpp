#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "gen/TestData_DCPS.hpp"
#include "../../interface/sub_interface.hpp"


class TestSubscriber: public TestMiddlewareSub<TestDataType>{
public:
    TestSubscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_priority):
            TestMiddlewareSub<TestDataType>(topic, msgCount, prior, cpu_index, filename, topic_priority),
            _dp(org::opensplice::domain::default_id()),
            _provider("file://QoS.xml", "TestProfile"),
            _topic(_dp, topic, _provider.topic_qos()),
            _subscriber(_dp),
            _dr(_subscriber, _topic, _provider.datareader_qos())
            {}


    bool receive() override {
        auto start_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        auto samples = _dr.select().max_samples(1).state(dds::sub::status::DataState::new_data()).take();
        unsigned long proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - start_timestamp;
        if(samples.length() > 0){
            auto msg = samples.begin()->data();
            write_received_msg(msg, proc_time);
            //std::cout<< msg.id()<<std::endl;
            return true;
        }
        return false;
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