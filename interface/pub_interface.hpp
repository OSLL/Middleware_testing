#include <string>
#include <chrono>
#include <ctime>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <thread>
#include <nlohmann/json.hpp>
#include "test_errors.hpp"
#include <sys/stat.h>
#include <errno.h>

#define CPUSET_MODE_T (S_IWUSR|S_IRUSR|S_IWGRP|S_IRGRP|S_IWOTH|S_IROTH)

class TestMiddlewarePub {
public:
    explicit TestMiddlewarePub(std::string &topic,  int msgCount, int prior, int cpu_index,
            int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step,
            std::string &filename, int topic_priority) :
    _filename(filename),
    _topic_name(topic),
    _msInterval(interval),
    _msgCount(msgCount),
    _priority(prior),
    _cpu_index(cpu_index),
    _byteSizeMin(min_msg_size),
    _byteSizeMax(max_msg_size),
    _step(step),
    _msg_count_befor_step(msgs_before_step),
    _topic_priority(topic_priority),
    _write_msg_time(msgCount)
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
            int err = mkdir("/sys/fs/cgroup/cpuset/pub_cpuset", CPUSET_MODE_T);
            if(errno != EEXIST && err != 0)
                throw test_exception("Error in adding to cpuset!", CPUSET_ERROR);
            std::ofstream f_cpu("/sys/fs/cgroup/cpuset/pub_cpuset/cpuset.cpus", std::ios_base::out);
            if(!f_cpu.is_open()){
                throw test_exception("Error in adding to cpuset!", CPUSET_ERROR);
            }
            f_cpu.write(std::to_string(cpu_index).c_str(), std::to_string(cpu_index).size());
            f_cpu.close();
            std::ofstream f_exclusive("/sys/fs/cgroup/cpuset/pub_cpuset/cpuset.cpu_exclusive", std::ios_base::out);
            if(!f_exclusive.is_open()){
                throw test_exception("Error in adding to cpuset!", CPUSET_ERROR);
            }
            f_exclusive.write("1", 1);
            f_exclusive.close();
            std::ofstream f_mem("/sys/fs/cgroup/cpuset/pub_cpuset/cpuset.mems", std::ios_base::out);
            if(!f_mem.is_open()){
                throw test_exception("Error in adding to cpuset!", CPUSET_ERROR);
            }
            f_mem.write("0", 1);
            f_mem.close();
            std::ofstream f_task("/sys/fs/cgroup/cpuset/pub_cpuset/tasks", std::ios_base::out);
            if(!f_task.is_open()){
                throw test_exception("Error in adding to cpuset!", CPUSET_ERROR);
            }
            else {
                auto s = std::to_string(id);
                f_task.write(s.c_str(),s.length());
            }
            f_task.close();
        }
    };
    int StartTest(){

        unsigned long proc_time = 0;
        std::this_thread::sleep_for(std::chrono::seconds(4));

        int cur_size = _byteSizeMin;

        for (auto i = 0; i < _msgCount; ++i) {

            if(i % (_msg_count_befor_step-1) == 0 && cur_size <= _byteSizeMax)
                cur_size += _step;

            proc_time = publish(i, cur_size);

            if(proc_time == 0)
                throw test_exception("Processing time hasn't been written!", TEST_ERROR);

            _write_msg_time[i] = proc_time;

            std::this_thread::sleep_for(std::chrono::milliseconds(_msInterval));
        }

        std::string end_str;
        std::cin >> end_str;

        if(end_str == "end") {
            to_Json();
            return 0;
        }

        std::this_thread::sleep_for(std::chrono::seconds(20));

        return -1;
    }

    void to_Json(){
        auto json = nlohmann::json::array();

        for (int i = 0; i < _msgCount; ++i) {

            nlohmann::json msg;
            msg["msg"] =
                    {
                        {"id", i},
                        {"proc_time", _write_msg_time[i]}
                    };

            json.push_back(msg);
        }

        std::ofstream file(_filename);
        file << json;
    }

    virtual unsigned long publish(short id, unsigned size)=0;

protected:
    std::string _filename;
    std::string _topic_name;
    int _msInterval;
    int _msgCount;
    int _priority; //not stated
    int _cpu_index; //not stated
    int _byteSizeMin;
    int _byteSizeMax;
    int _step;
    int _msg_count_befor_step;
    int _topic_priority;
    std::vector <unsigned long> _write_msg_time;

};