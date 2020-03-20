#include <string>
#include <chrono>
#include <ctime>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "nlohmann/json.hpp"
#include <fstream>

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
    int StartTest(){
        std::this_thread::sleep_for(std::chrono::seconds(4));
        std::string id = "id";
        std::string time = "time";
        std::string end = "end";
        int cur_size = _byteSizeMin;
        for (int i = 0; i < _msgCount; ++i) {
            unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            std::string msg = id + std::to_string(i) + time + std::to_string(cur_time) + end + std::string('a', cur_size - 30);
            if(i % (_msg_count_befor_step-1) == 0 && cur_size <= _byteSizeMax)
                cur_size += _step;
            publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(_msInterval));
        }
        return 0;
    }

    virtual void publish(std::string msg)=0;

    virtual void setQoS(std::string filename)=0;	//считывать наверное тоже из json, так как будут разные конфигурации QoS

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
    explicit TestMiddlewareSub(std::string topic, int msgCount=0, int prior = -1, int cpu_index = -1) :
    rec_time(msgCount),
    msgs(msgCount),
    _msgCount(msgCount),
    _priority(prior),
    _cpu_index(cpu_index)
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

    virtual int receive()=0;  //возвращает вектор принятых сообщений

    int StartTest(){
        int count = 0;
        while (count < _msgCount){
            count += receive();
        }
        to_Json();
        return 0;
    }

    void to_Json(){
        nlohmann::json json = nlohmann::json::array();
        for (int i = 0; i < _msgCount; ++i) {
            auto time_pos = msgs[i].find("time")+ sizeof("time");
            auto end_pos = msgs[i].find("end");
            if(time_pos == std::string::npos || end_pos==std::string::npos ) {
                std::cout << "Msg parse error!" << std::endl;
                continue;
            }
            nlohmann::json msg;
            std::string id = msgs[i].substr(sizeof("id") - 1, time_pos - sizeof("time") - sizeof("id") + 1);
            std::string time = msgs[i].substr(time_pos - 1, end_pos-time_pos + 1);
            msg["msg"] = {{"id", std::stol(id)}, {"sent_time", std::stol(time)}, {"rec_time", rec_time[i]}, {"delay", std::stol(time) - rec_time[i]}};
            json.push_back(msg);
        }
        std::ofstream file("key.json");
        file << json;
    }

    virtual void setQoS(std::string filename)=0;

protected:
    std::vector<unsigned long> rec_time;
    std::vector<std::string> msgs;
    int _msgCount;
    int _priority; //def not stated
    int _cpu_index; //def not stated
};