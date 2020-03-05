#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include <unistd.h>
#include "gen/TestData_DCPS.hpp"


class TestPublisher{
    dds::domain::DomainParticipant dp;
    dds::topic::Topic <TestDataType> topic;
    dds::pub::Publisher pub;
    dds::pub::DataWriter <TestDataType> dw;
    dds::core::QosProvider provider;
    sched_param priority;
    std::vector<unsigned long int> pub_time;

public:
    TestPublisher(): dp(org::opensplice::domain::default_id()),
                        topic(dp, "TestTopic"),
                        pub(dp),
                        dw(pub, topic),
                        provider("file://QoS.xml", "TestProfile"),
                        pub_time(5000)
                        {
                            pid_t id = getpid();
                            std::ofstream f_task("/sys/fs/cgroup/cpuset/pub_cpuset/tasks", std::ios_base::out);
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
        TestDataType msg(std::vector<char>(60000));
        std::this_thread::sleep_for(std::chrono::seconds(4));
        for(int i = 0; i < count; i++){
            pub_time[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            dw.write(msg);
        }
        return 0;
    }
    ~TestPublisher(){
        std::ofstream file2("publisher.txt");
        for(unsigned i=0; i<pub_time.size();i++)
            file2 << pub_time[i] << ' ';
        file2 << std::endl;
        file2.close();
    }
};


int main(int argc, char **argv) {
    try {
        TestPublisher publisher;
        publisher.StartTest();
    }
    catch (...){
        return -1;
    }
}