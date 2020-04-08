#include <string>
#include <chrono>
#include <ctime>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "../nlohmann/json.hpp"
#include <fstream>
#include <cmath>
#include "test_errors.hpp"

#define TIMEOUT 2 * pow(10, 10)

class TestMiddlewareSub
{
public:
    explicit TestMiddlewareSub(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename,
            int topic_priority, bool isMsgProcTimeTest) :
            _topic_name(topic),
            rec_time(msgCount),
            msgs(msgCount),
            _read_msg_time(msgCount),
            _topic_priority(topic_priority),
            _msgCount(msgCount),
            _priority(prior),
            _cpu_index(cpu_index),
            _filename(filename),
            _isMsgProcTimeTest(isMsgProcTimeTest)
    {
        pid_t id = getpid();
        if(prior >= 0){
            sched_param priority;
            priority.sched_priority = _priority;
            int err = sched_setscheduler(id, SCHED_FIFO, &priority);
            if(err) {
                throw test_exception("Error in setting priority: " + std::to_string(err), THREAD_PRIOR_ERROR);
            }
        }
        if(cpu_index >= 0){
            std::ofstream f_task("/sys/fs/cgroup/cpuset/sub_cpuset/tasks", std::ios_base::out);
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

    virtual int receive()=0;  //записывает вектор принятых сообщений

    int StartTest(){
        int count = 0;
        unsigned long start_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        while (count < _msgCount) {
            int pre_rec_count = count;
            count += receive();
            unsigned long end_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            if (pre_rec_count == count && count != 0) {
                if (end_timeout - start_timeout > TIMEOUT)
                    throw test_exception("Timeout exceeded!", TIMEOUT_ERROR);
            } else {
                start_timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            }
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
            if(_isMsgProcTimeTest)
                msg["msg"] = {{"id", id}, {"proc_time", _read_msg_time[i]}};
            else
                msg["msg"] = {{"id", id}, {"sent_time", sent_time}, {"rec_time", rec_time[i]}, {"delay", rec_time[i] - sent_time}};
            json.push_back(msg);
        }
        std::ofstream file(_filename);
        file << json;
    }

protected:
    std::string _topic_name;
    std::vector<unsigned long> rec_time;
    std::vector<std::pair<short, unsigned long>> msgs;
    std::vector <unsigned long> _read_msg_time;
    int _topic_priority;
    int _msgCount;
    int _priority; //def not stated
    int _cpu_index; //def not stated
    std::string _filename;
    bool _isMsgProcTimeTest;
};
