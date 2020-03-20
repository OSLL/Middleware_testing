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
    TestSubscriber(std::string topic,  int msgCount=0, int prior=-1, int cpu_index=-1):
            TestMiddlewareSub(topic, msgCount, prior, cpu_index),
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
                    msgs[i]=std::string(j->data().data().begin(), j->data().data().end());
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
    try {
        TestSubscriber subscriber("/topic", 500);
        subscriber.StartTest();
    }
    catch (...){
        std::cout<< "Error!\n";
        return -1;
    }
}