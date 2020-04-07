#include <string>
#include <chrono>
#include <ctime>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "../nlohmann/json.hpp"
#include <fstream>
#include <thread>
#include "test_errors.hpp"

class TestMiddlewarePub
{
public:
    explicit TestMiddlewarePub(std::string &topic,  int msgCount, int prior, int cpu_index,
            int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step) :
            _topic_name(topic),
    _msInterval(interval),
    _msgCount(msgCount),
    _priority(prior),
    _cpu_index(cpu_index),
    _byteSizeMin(min_msg_size),
    _byteSizeMax(max_msg_size),
    _step(step),
    _msg_count_befor_step(msgs_before_step) {

        std::cout << "topic: " << _topic_names[0] << std::endl;
        std::cout << "interval: " << _msInterval << std::endl;
        std::cout << "msg_count: " <<_msgCount << std::endl;
        std::cout << "priority: " << _priority << std::endl;
        std::cout << "cpu_index: " << _cpu_index << std::endl;
        std::cout << "min_byte: " << _byteSizeMin << std::endl;
        std::cout << "max_byte: " << _byteSizeMax << std::endl;
        std::cout << "step :" << _step << std::endl;
        std::cout << "count_before_step :" << _msg_count_befor_step << std::endl;

        pid_t id = getpid();
        if(prior >= 0){
            sched_param priority;
            priority.sched_priority = _priority;
            int err = sched_setscheduler(id, SCHED_FIFO, &priority);
            if(err) {
                std::cout << prior << std::endl;
                std::cout << "Erorr in setting priority: " << -err << std::endl;
                throw test_exception("Error in setting priority: " + std::to_string(err), THREAD_PRIOR_ERROR);
            }
        }
        if(cpu_index >= 0){
            std::ofstream f_task("/sys/fs/cgroup/cpuset/pub_cpuset/tasks", std::ios_base::out);
            if(!f_task.is_open()){
                throw test_exception("Error in adding to cpuset!", CPUSET_ERROR);
            }
            else{                                                   // добавить изменения номера ядра для привязки
                auto s = std::to_string(id);
                f_task.write(s.c_str(),s.length());
            }
            f_task.close();
        }
    };

    int StartTest(){
        std::this_thread::sleep_for(std::chrono::seconds(4));
        int cur_size = _byteSizeMin;
        for (int i = 0; i < _msgCount; ++i) {
            if(i % (_msg_count_befor_step-1) == 0 && cur_size <= _byteSizeMax)
                cur_size += _step;
            publish(i, cur_size);
            std::this_thread::sleep_for(std::chrono::milliseconds(_msInterval));
        }
        std::string end_str;
        std::cin >> end_str;
        if(end_str == "end")
            return 0;
        std::this_thread::sleep_for(std::chrono::seconds(20));
        return -1;
    }

    virtual void publish(short id, unsigned size)=0;

    virtual void setQoS(std::string filename)=0;	//считывать наверное тоже из json, так как будут разные конфигурации QoS

protected:
    std::string _topic_name;
    int _msInterval;
    int _msgCount;
    int _priority; //not stated
    int _cpu_index; //not stated
    int _byteSizeMin;
    int _byteSizeMax;
    int _step;
    int _msg_count_befor_step;
};

