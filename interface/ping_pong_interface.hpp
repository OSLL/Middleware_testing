#include <string>
#include <chrono>
#include <ctime>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <thread>
#include <nlohmann/json.hpp>
#include <sys/stat.h>
#include <errno.h>
#include "test_errors.hpp"

#define CPUSET_MODE_T (S_IWUSR|S_IRUSR|S_IWGRP|S_IRGRP|S_IWOTH|S_IROTH)
#define TIMEOUT 2 * pow(10, 10)

template <class MsgType>
class TestMiddlewarePingPong {
public:
    TestMiddlewarePingPong(
            std::string &topic, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst) :
            _topic_name(topic),
            _recieve_timestamps(msgCount),
            _msgs(msgCount),
            _topic_priority(topic_priority),
            _msgCount(msgCount),
            _priority(prior),
            _cpu_index(cpu_index),
            _filename(filename),
            _msInterval(msInterval),
            _msgSize(msgSize)
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
            int err = mkdir("/sys/fs/cgroup/cpuset/sub_cpuset", CPUSET_MODE_T);
            if(errno != EEXIST && err != 0)
                throw test_exception("Error in adding to cpuset!", errno);
            std::ofstream f_cpu("/sys/fs/cgroup/cpuset/sub_cpuset/cpuset.cpus", std::ios_base::out);
            if(!f_cpu.is_open()){
                throw test_exception("Error in adding to cpuset!", CPUSET_ERROR);
            }
            f_cpu.write(std::to_string(cpu_index).c_str(), std::to_string(cpu_index).size());
            f_cpu.close();

            std::ofstream f_exclusive("/sys/fs/cgroup/cpuset/sub_cpuset/cpuset.cpu_exclusive", std::ios_base::out);
            if(!f_exclusive.is_open()){
                throw test_exception("Error in adding to cpuset!", CPUSET_ERROR);
            }
            f_exclusive.write("1", 1);
            f_exclusive.close();
            std::ofstream f_mem("/sys/fs/cgroup/cpuset/sub_cpuset/cpuset.mems", std::ios_base::out);
            if(!f_mem.is_open()){
                throw test_exception("Error in adding to cpuset!", CPUSET_ERROR);
            }
            f_mem.write("0", 1);
            f_mem.close();
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

    void write_received_msg(MsgType &msg) {
        _msgs[get_id(msg)] = msg;
        _recieve_timestamps[get_id(msg)] = std::chrono::duration_cast<std::chrono::
        nanoseconds>(std::chrono::high_resolution_clock::
                     now().time_since_epoch()).count();
    };

    virtual bool receive() = 0;

    int StartTest(){
        bool isTimeoutEx = false;
        unsigned long start_timeout, end_timeout;
        std::this_thread::sleep_for(std::chrono::seconds(4));
        start_timeout = end_timeout = std::chrono::duration_cast<std::chrono::
        nanoseconds>(std::chrono::high_resolution_clock::
                     now().time_since_epoch()).count();

        for(int i = 0; i < _msgCount; i++){
            if(_isFirst)
                publish(i, _msgSize);
            while (true) {
                // true - принято
                if(receive()) {
                    start_timeout = std::chrono::duration_cast<std::chrono::
                    nanoseconds>(std::chrono::high_resolution_clock::
                                 now().time_since_epoch()).count();
                    break;
                } else {
                    end_timeout = std::chrono::duration_cast<std::chrono::
                    nanoseconds>(std::chrono::high_resolution_clock::
                                 now().time_since_epoch()).count();
                    if (end_timeout - start_timeout > TIMEOUT) {
                        isTimeoutEx = true;
                        break;
                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            if(isTimeoutEx)
                break;
            if(_isFirst)
                continue;
            publish(i, _msgSize);

        }

        to_json();
        if(isTimeoutEx)
            return TEST_ERROR;
        return 0;
    }
    virtual short get_id(MsgType &msg) = 0;
    virtual unsigned long get_timestamp(MsgType &msg) = 0;

    void to_json() {
        auto json_output = nlohmann::json::array();
        nlohmann::json json_msg;

        for (unsigned i = 0; i < _msgs.size(); i++) {
            auto msg = _msgs[i];

            json_msg["msg"] =
                    {
                            {"id", get_id(msg)},
                            {"sent_time", get_timestamp(msg)},
                            {"recieve_timestamp", _recieve_timestamps[i]},
                            {"delay", _recieve_timestamps[i] - get_timestamp(msg)},
                    };
            json_output.emplace_back(json_msg);
        }

        std::ofstream file(_filename);
        file << json_output;
    }

    virtual void publish(short id, unsigned size)=0;

protected:
    std::string _topic_name;
    std::vector<unsigned long> _recieve_timestamps;
    std::vector<MsgType> _msgs;
    int _topic_priority;
    int _msgCount;
    int _priority; //def not stated
    int _cpu_index; //def not stated
    std::string _filename;
    bool _isFirst;
    int _msInterval;
    int _msgSize;
};
