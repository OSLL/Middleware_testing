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

    virtual bool receive(MsgType &msg) {
        _msgs.emplace_back(msg);
        _recieve_timestamps.emplace_back(std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count());
        return true;
    };

    int StartTest(){

        MsgType msg;
        int count = 0;
        int res_code = 0;
        unsigned long timestamp = 0;

        unsigned long start_timeout, end_timeout;
        start_timeout = end_timeout = std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count();

        while (true) {

            // true - принято
            if(receive(msg)) {
                start_timeout = std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                             now().time_since_epoch()).count();
            } else {
                end_timeout = std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                             now().time_since_epoch()).count();
            }

            if (end_timeout - start_timeout > TIMEOUT)
                break;

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        to_json();

        return 0;
    }

    void to_json() {

        auto json_output = nlohmann::json::array();
        nlohmann::json json_msg;

        for (int i = 0; i < _msgs.size(); i++) {
            auto msg = _msgs[i];

            json_msg["msg"] =
                    {
                        {"id", msg.id},
                        {"sent_time", msg.timestamp},
                        {"recieve_timestamp", _recieve_timestamps[i]},
                        {"delay", _recieve_timestamps[i] - msg.timestamp},
                        {"proc_time", msg.processing_time}
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
