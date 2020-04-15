#include <string>
#include <chrono>
#include <ctime>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "nlohmann/json.hpp"
#include <fstream>
#include <cmath>
#include "test_errors.hpp"
#include <thread>

#define TIMEOUT 2 * pow(10, 10)

template <class MsgType>
class TestMiddlewareSub {

public:
    TestMiddlewareSub(
            std::string &topic, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority) :
            _topic_name(topic),
            _recieve_timestamps(msgCount),
            _msgs(msgCount),
            _read_msg_time(msgCount),
            _topic_priority(topic_priority),
            _msgCount(msgCount),
            _priority(prior),
            _cpu_index(cpu_index),
            _filename(filename) {
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

    void write_received_msg(MsgType &msg, unsigned long proc_time) {
        _msgs[get_id(msg)] = msg;
        _recieve_timestamps[get_id(msg)] = std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count();
        _read_msg_time[get_id(msg)] = proc_time;
    };

    virtual bool receive() = 0;

    int StartTest(){
        unsigned long start_timeout, end_timeout;
        start_timeout = end_timeout = std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count();
        while (true) {
            // true - принято
            if(receive()) {
                start_timeout = std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                             now().time_since_epoch()).count();

            } else {
                end_timeout = std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                             now().time_since_epoch()).count();
                if (end_timeout - start_timeout > TIMEOUT)
                    break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        }

        to_json();
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
                        {"read_proc_time", _read_msg_time[i]}
                    };

            json_output.emplace_back(json_msg);
        }

        std::ofstream file(_filename);
        file << json_output;
    }

protected:
    std::string _topic_name;
    std::vector<unsigned long> _recieve_timestamps;
    std::vector<MsgType> _msgs;
    std::vector <unsigned long> _read_msg_time;
    int _topic_priority;
    int _msgCount;
    int _priority; //def not stated
    int _cpu_index; //def not stated
    std::string _filename;
};
