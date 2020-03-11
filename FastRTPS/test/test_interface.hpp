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

template <class Publisher>
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
    _msg_count_befor_step(msgs_before_step),
    _topic(topic)
    {
//        _publisher = this->createPublisher(topic);
        pid_t id = getpid();
        if(prior != -1){
            sched_param priority;
            priority.sched_priority = sched_get_priority_max(prior);
            int err = sched_setscheduler(id, SCHED_FIFO, &priority);
            if(err)
                std::cout << "Erorr in setting priority: "<< -err << std::endl;
        }
        if(cpu_index != -1){
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

    virtual Publisher* createPublisher(std::string topic)=0;

    virtual void publish(std::string &msg)=0;

    void test() {
	_publisher = createPublisher(_topic);
	std::string msg(_byteSizeMax, 'a');
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	for(int i=0; i<_msgCount; ++i) {
	    std::stringstream ss;
	    ss << "id" << i << "time" << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() << "end";
	    msg.replace(0, msg.find("end") + sizeof("end")-1, ss.str());
	    msg.resize(_byteSizeMax);
	    publish(msg);
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
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
    std::string _topic;
    Publisher* _publisher;
};

template <class Subscriber>
class TestMiddlewareSub
{
public:

    explicit TestMiddlewareSub(std::string topic, int msgCount=0, int prior = -1, int cpu_index = -1, int max_msg_size=64000) :
    rec_time(msgCount),
    msgs(msgCount),
    _msgCount(msgCount),
    _priority(prior),
    _cpu_index(cpu_index),
    _topic(topic),
    _byteSizeMax(max_msg_size)
    {
//        _subscriber = createSubscriber(topic);
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

    virtual Subscriber* createSubscriber(std::string topic)=0;

    virtual std::vector<std::string> receive()=0;  //возвращает вектор принятых сообщений

    void to_Json(){
        nlohmann::json json = nlohmann::json::array();
        for (int i = 0; i < _msgCount; ++i) {
            auto time_pos = msgs[i].find("time")+ sizeof("time");
            auto end_pos = msgs[i].find("end");
            if(time_pos == std::string::npos || end_pos==std::string::npos ) {
                std::cout << "Msg error!" << std::endl;
                continue;
            }
            nlohmann::json msg;
            std::string id = msgs[i].substr(sizeof("id") - 1, time_pos - sizeof("time") - sizeof("id") + 1);
            std::string time = msgs[i].substr(time_pos - 1, end_pos-time_pos + 1);
            msg["msg"] = {{"id", std::stoi(id)}, {"sent_time", std::stoul(time)}, {"rec_time", rec_time[i]}};
            json.push_back(msg);
        }
        std::ofstream file("key.json");
        file << json;
    }

//    virtual void setQoS(std::string filename)=0;


    void test() {
	_subscriber = createSubscriber(_topic);
	std::tie(msgs, rec_time) = _subscriber->receive();
	to_Json();
    }
protected:
    std::vector<unsigned long> rec_time;
    std::vector<std::string> msgs;
    int _msgCount;
    int _priority; //def not stated
    int _cpu_index; //def not stated
    int _byteSizeMax;
    std::string _topic;
    Subscriber* _subscriber;

};
