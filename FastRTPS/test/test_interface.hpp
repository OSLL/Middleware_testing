#pragma once
#include <string>
#include <chrono>
#include <ctime>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "nlohmann/json.hpp"
#include <fstream>
#include <thread>
#include <sstream>

class TestMiddlewarePub
{
public:
 
    explicit TestMiddlewarePub(std::string topic,  int msgCount=0, int prior = -1, int cpu_index = -1,
            int min_msg_size=0, int max_msg_size=64000, int step=0, int interval = 0, int msgs_before_step = 100) :
    _msInterval(interval),
    _msgCount(msgCount),
    _priority(prior),
    _cpu_index(cpu_index),
    _byteSizeMin(min_msg_size),
    _byteSizeMax(max_msg_size),
    _step(step),
    _msg_count_befor_step(msgs_before_step)
    {
        pid_t id = getpid();
        if(prior >= 0){
            sched_param priority;
            priority.sched_priority = sched_get_priority_max(prior);
            int err = sched_setscheduler(id, SCHED_FIFO, &priority);
            if(err)
                std::cout << "Erorr in setting priority: "<< -err << std::endl;
        }
        if(cpu_index >= 0){
            std::ofstream f_task("/sys/fs/cgroup/cpuset/pub_cpuset/tasks", std::ios_base::out);
            if(!f_task.is_open()){
                std::cout << "Erorr in adding to cpuset"<< std::endl;
            }
            else{                                                   // добавить изменения номера ядра для привязки
                auto s = std::to_string(id);
                f_task.write(s.c_str(),s.length());
            }
            f_task.close();
        }
    };

    virtual void publish(short id, unsigned size)=0;

    virtual void setQoS(std::string filename)=0;

    int StartTest(){
        std::this_thread::sleep_for(std::chrono::seconds(4));
        int cur_size = _byteSizeMin;
        for (int i = 0; i < _msgCount; ++i) {
            if(i % (_msg_count_befor_step-1) == 0 && cur_size <= _byteSizeMax)
                cur_size += _step;
            publish(i, cur_size);
            std::this_thread::sleep_for(std::chrono::milliseconds(_msInterval));
        }
	std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        return 0;
    }

protected:
    int _msInterval;
    int _msgCount;
    int _priority; //not stated
    int _cpu_index; //not stated
    int _byteSizeMin;
    int _byteSizeMax;
    int _step;
    int _msg_count_befor_step;
};


class TestMiddlewareSub
{
public:

    explicit TestMiddlewareSub(std::string topic, int msgCount=0, int prior = -1, int cpu_index = -1, int max_msg_size=64000, std::string res_filename="res.json") :
    rec_time(msgCount),
    msgs(msgCount),
    _msgCount(msgCount),
    _priority(prior),
    _cpu_index(cpu_index),
    _byteSizeMax(max_msg_size),
    _filename(res_filename)
    {
        pid_t id = getpid();
        if(prior != -1){
            sched_param priority;
            priority.sched_priority = sched_get_priority_max(prior);
            int err = sched_setscheduler(id, SCHED_FIFO, &priority);
            if(err)
                std::cout << "Erorr in setting priority: "<< -err << std::endl;
        }
        if(cpu_index != -1){
            std::ofstream f_task("/sys/fs/cgroup/cpuset/sub_cpuset/tasks", std::ios_base::out);
            if(!f_task.is_open()){
                std::cout << "Erorr in adding to cpuset"<< std::endl;
            }
            else{                                                   // добавить изменения номера ядра для привязки
                auto s = std::to_string(id);
                f_task.write(s.c_str(),s.length());
            }
            f_task.close();
        }
    };

    virtual int receive()=0;

    void to_Json(){
        auto json = nlohmann::json::array();
        for (int i = 0; i < _msgCount; ++i) {
            nlohmann::json msg;
            auto id = msgs[i].first;
            auto sent_time = msgs[i].second;
            msg["msg"] = {{"id", id}, {"sent_time", sent_time}, {"rec_time", rec_time[i]}, {"delay", sent_time - rec_time[i]}};
            json.push_back(msg);
        }
        std::ofstream file(_filename);
        file << json;
    }

    virtual void setQoS(std::string filename)=0;

    void test() {
        int count = 0;
        while (count < _msgCount){
            count += receive();
        }
        to_Json();
    }
protected:
    std::vector<unsigned long> rec_time;
    std::vector<std::pair<short, unsigned long>> msgs;
    int _msgCount;
    int _priority; //def not stated
    int _cpu_index; //def not stated
    int _byteSizeMax;
    std::string _filename;
};
