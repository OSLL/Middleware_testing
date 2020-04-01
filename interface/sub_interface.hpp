#include <string>
#include <chrono>
#include <ctime>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "../nlohmann/json.hpp"
#include <fstream>

class TestMiddlewareSub
{
public:
    explicit TestMiddlewareSub(std::vector<std::string> &topics, int msgCount, int prior, int cpu_index, std::string filename) :
            _topic_names(topics),
            rec_time(msgCount),
            msgs(msgCount),
            _msgCount(msgCount),
            _priority(prior),
            _cpu_index(cpu_index),
            _filename(filename)
    {
        pid_t id = getpid();
        if(prior >= 0){
            sched_param priority;
            priority.sched_priority = sched_get_priority_max(prior);
            int err = sched_setscheduler(id, SCHED_FIFO, &priority);
            if(err) {
                std::cout << "Error in setting priority: " << -err << std::endl;
                throw;
            }
        }
        if(cpu_index >= 0){
            std::ofstream f_task("/sys/fs/cgroup/cpuset/sub_cpuset/tasks", std::ios_base::out);
            if(!f_task.is_open()){
                std::cout << "Error in adding to cpuset"<< std::endl;
                throw;
            }
            else{                                                   // добавить изменения номера ядра для привязки
                auto s = std::to_string(id);
                f_task.write(s.c_str(),s.length());
            }
            f_task.close();
        }
    };

    virtual int receive(std::string &topic)=0;  //записывает вектор принятых сообщений

    int StartTest(){
        int count = 0;
        while (count < _msgCount){
            count += receive(_topic_names[0]);
        }
        to_Json();
        return 0;
    }

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

protected:
    std::vector<std::string> _topic_names;
    std::vector<unsigned long> rec_time;
    std::vector<std::pair<short, unsigned long>> msgs;
    int _msgCount;
    int _priority; //def not stated
    int _cpu_index; //def not stated
    std::string _filename;
};