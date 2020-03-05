#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include <unistd.h>
#include "gen/TestData_DCPS.hpp"


class TestSubscriber{
    dds::domain::DomainParticipant dp;
    dds::topic::Topic <TestDataType> topic;
    dds::sub::Subscriber sub;
    dds::sub::DataReader <TestDataType> dr;
    dds::core::QosProvider provider;
    sched_param priority;
    std::vector<unsigned long int> sub_time;

public:
    TestSubscriber(): dp(org::opensplice::domain::default_id()),
                        topic(dp, "TestTopic"),
                        sub(dp),
                        dr(sub, topic),
                        provider("file://QoS.xml", "TestProfile"),
                        sub_time(5000)
                        {
                            pid_t id = getpid();
                            std::ofstream f_task("/sys/fs/cgroup/cpuset/sub_cpuset/tasks", std::ios_base::out);
                            if(!f_task.is_open()){
                                std::cout << "Erorr in adding to cpuset"<< std::endl;
                            }
                            else{
                                auto s = std::to_string(id);
                                f_task.write(s.c_str(),s.length());
                            }
                            f_task.close();
                            priority.sched_priority = sched_get_priority_max(SCHED_FIFO);
                            int err = sched_setscheduler(id, SCHED_FIFO, &priority);
                            if(err)
                                std::cout << "Erorr in setting priority: "<< -err << std::endl;
                        }
    int StartTest(int count = 5000){
        int i = 0;
        dds::core::Time time;
        dds::sub::SampleInfo info;
        while(i < count){
            auto samples = dr.read();
            for(auto j=samples.begin(); j != samples.end(); ++j){
                sub_time[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
                //info = j->info();
                //time = info.timestamp();
                //pub_time[i] = time.nanosec();
                i++;
            }
        }
        return 0;
    }

    ~TestSubscriber(){
        std::ofstream file1("subscriber.txt", std::ofstream::app);
        for(unsigned i=0; i<sub_time.size();i++)
            file1 << sub_time[i] << ' ';
        file1 << std::endl;
        file1.close();
    }
};

int main(int argc, char **argv) {
    try {
        TestSubscriber subscriber;
        return subscriber.StartTest();

    }
    catch (...){
        return -1;
    }
}