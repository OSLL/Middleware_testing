#include <string>
#include <chrono>
#include <ctime>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <fstream>
#include <thread>
#include <future>
#include <nlohmann/json.hpp>
#include <sys/stat.h>
#include <errno.h>
#include "test_errors.hpp"

#define CPUSET_MODE_T (S_IWUSR|S_IRUSR|S_IWGRP|S_IRGRP|S_IWOTH|S_IROTH)
#define WAIT_MSG_TIMEOUT 2 * pow(10, 10)
#define WATERMARK 50

template <class MsgType>
class TestMiddlewarePingPong {
public:
    TestMiddlewarePingPong(
            std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst) :
            _topic_name1(topic1),
            _topic_name2(topic2),
            _filename(filename),
            _recieve_timestamps(msgCount),
            _read_msg_time(msgCount),
            _write_msg_time(msgCount),
            _msgs(msgCount),
            _isFirst(isFirst),
            _isNew(false),
            _topic_priority(topic_priority),
            _msgCount(msgCount),
            _priority(prior),
            _cpu_index(cpu_index),
            _msInterval(msInterval),
            _msgSize(msgSize),
            _cur_size(_msgSizeMin)
            {
                set_cpu_index_and_prior();
            }

    TestMiddlewarePingPong(
            std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSizeMin, int msgSizeMax, int step, int before_step, bool isFirst) :
            _topic_name1(topic1),
            _topic_name2(topic2),
            _filename(filename),
            _recieve_timestamps(msgCount),
            _read_msg_time(msgCount),
            _write_msg_time(msgCount),
            _msgs(msgCount),
            _isFirst(isFirst),
            _isNew(true),
            _topic_priority(topic_priority),
            _msgCount(msgCount),
            _priority(prior),
            _cpu_index(cpu_index),
            _msInterval(msInterval),
            _msgSizeMin(msgSizeMin),
            _msgSizeMax(msgSizeMax),
            _step(step),
            _msgs_before_step(before_step),
            _msgSize(msgSizeMin),
            _cur_size(_msgSizeMin)
            {
                if (_msgSize == 0){
                    _msInterval = 0;
                    _msgSize = _msgSizeMax;
                    _msgSizeMin = _msgSize;
                    _constQueue = true;
                }
                set_cpu_index_and_prior();
            }
    
    void set_cpu_index_and_prior(){
        pid_t id = getpid();
        if(_priority >= 0){
            sched_param priority;
            priority.sched_priority = _priority;
            int err = sched_setscheduler(id, SCHED_FIFO, &priority);
            if(err) {
                throw test_exception("Error in setting priority: " + std::to_string(err), THREAD_PRIOR_ERROR);
            }
        }
        if(_cpu_index >= 0){
            int err = mkdir("/sys/fs/cgroup/cpuset/sub_cpuset", CPUSET_MODE_T);
            if(errno != EEXIST && err != 0)
                throw test_exception("Error in adding to cpuset!", errno);
            std::ofstream f_cpu("/sys/fs/cgroup/cpuset/sub_cpuset/cpuset.cpus", std::ios_base::out);
            if(!f_cpu.is_open()){
                throw test_exception("Error in adding to cpuset!", CPUSET_ERROR);
            }
            f_cpu.write(std::to_string(_cpu_index).c_str(), std::to_string(_cpu_index).size());
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

    int StartTestOld(){
        bool isTimeoutEx = false;

        std::this_thread::sleep_for(std::chrono::seconds(4));

        for(int i = 0; i < _msgCount; i++) {
            if (_isFirst) {
                publish(i, _msgSize);
            }
            isTimeoutEx = wait_for_msg();
            if (isTimeoutEx)
                break;
            if (!_isFirst) {
                publish(i, _msgSize);
            }
        }

        std::cout << "Test ended" << std::endl;

        to_json();

        if(isTimeoutEx)
            return TEST_ERROR;
        return 0;
    }

    bool wait_for_msg(){        //func waits for TIMEOUT to receive msgs
        unsigned long start_timeout, end_timeout;
        start_timeout = end_timeout = std::chrono::duration_cast<std::chrono::
        nanoseconds>(std::chrono::high_resolution_clock::
                     now().time_since_epoch()).count();
        while (_isFirst && _isNew) {
            mu.lock();
            bool isStarted = _isTestStarted;
            mu.unlock();
            if (!isStarted) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } else break;
        }
        bool notReceived = true;
        while (notReceived) {
            if (_last_rec_msg_id-1 == _msgCount && _isNew){
                break;
            }
            mu.lock();      // mute thread to write msg and update _last_rec_msg_id
            if(receive()) { // true - принято
                if(!_isNew || !_isFirst)
                    notReceived = false;
                start_timeout = std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                             now().time_since_epoch()).count();
            } else {
                end_timeout = std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                             now().time_since_epoch()).count();
                if (end_timeout - start_timeout > WAIT_MSG_TIMEOUT) {
                    mu.unlock();
                    return true;
                }

            }
            mu.unlock();

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    }

    int StartTestNew(){
        std::future<bool> future;
        std::this_thread::sleep_for(std::chrono::seconds(4));
        if (_isFirst)   //run receiving msgs in another thread
            future = std::async(std::launch::async, &TestMiddlewarePingPong<MsgType>::wait_for_msg, this);
        unsigned long start_timeout, end_timeout;
        start_timeout = end_timeout = std::chrono::duration_cast<std::chrono::
        nanoseconds>(std::chrono::high_resolution_clock::
                     now().time_since_epoch()).count();
        if (_isFirst) {
            for (auto i = 0; i < _msgCount; ++i) {
                if (_constQueue){
                    mu.lock();
                    if (i - _last_rec_msg_id > WATERMARK) {
                        --i;
                        mu.unlock();
                        end_timeout = std::chrono::duration_cast<std::chrono::
                        nanoseconds>(std::chrono::high_resolution_clock::
                                     now().time_since_epoch()).count();
                        if (end_timeout - start_timeout > WAIT_MSG_TIMEOUT)
                            break;
                        continue;
                    }
                    mu.unlock();
                }
                if (i == 0) {
                    mu.lock();
                    _isTestStarted = true;
                    mu.unlock();
                }
                start_timeout = std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                             now().time_since_epoch()).count();

                if (i % (_msgs_before_step - 1) == 0 && _cur_size <= _msgSizeMax)
                    _cur_size += _step;
                #ifdef MUTEX_PUBLISH
                mu.lock();
                publish(i, _cur_size);
                mu.unlock();
                #else
                publish(i, _cur_size);
                #endif


                std::this_thread::sleep_for(std::chrono::milliseconds(_msInterval));
            }
            std::cout << "Waitung for second thread!" << std::endl;
            future.wait();
            std::cout << "All threads done!" << std::endl;
        } else{
            while (!wait_for_msg());
            std::this_thread::sleep_for(std::chrono::seconds(4));
        }
        std::cout << "Test ended" << std::endl;

        to_json();
        return 0;
    }

    void write_received_msg(MsgType &msg) {
        _last_rec_msg_id = get_id(msg);
        _msgs[_last_rec_msg_id] = msg;
        _recieve_timestamps[_last_rec_msg_id] = std::chrono::duration_cast<std::chrono::
        nanoseconds>(std::chrono::high_resolution_clock::
                     now().time_since_epoch()).count();
        //std::cout << "rec" << _last_rec_msg_id  << std::endl;
        if (!_isFirst && _isNew){
            if (_last_rec_msg_id % (_msgs_before_step - 1) == 0 && _cur_size <= _msgSizeMax)
                _cur_size += _step;
            publish(_last_rec_msg_id, _cur_size);
        }
    };

    virtual bool receive() = 0;
    int StartTest(){
        if(_isNew) return StartTestNew();
        else return StartTestOld();
    }
    virtual short get_id(MsgType &msg) = 0;
    virtual unsigned long get_timestamp(MsgType &msg) = 0;

    void to_json() {
        auto json_output = nlohmann::json::array();
        nlohmann::json json_msg;
        std::cout << "Start writing to JSON" << std::endl;
        for (unsigned i = 0; i < _msgs.size(); i++) {
            auto &msg = _msgs[i];
            json_msg["msg"] =
                    {
                            {"id", get_id(msg)},
                            {"sent_time", get_timestamp(msg)},
                            {"recieve_timestamp", _recieve_timestamps[i]},
                            {"delay", _recieve_timestamps[i] - get_timestamp(msg)},
                            {"read_proc_time", _read_msg_time[i]},
                            {"proc_time", _write_msg_time[i]}
                    };
            json_output.emplace_back(json_msg);
        }

        std::ofstream file(_filename);
        if (!file){
            std::cout << "Cannot open file: "<< _filename << std::endl;
        }
        file << json_output;
        std::cout << "End writing to JSON, filename: "<< _filename << std::endl;
    }

    virtual void publish(short id, unsigned size)=0;

protected:
    std::string _topic_name1;
    std::string _topic_name2;
    std::string _filename;
    std::vector<unsigned long> _recieve_timestamps;
    std::vector <unsigned long> _read_msg_time;
    std::vector <unsigned long> _write_msg_time;
    std::vector<MsgType> _msgs;
    bool _isFirst;
    bool _isNew;
    bool _constQueue = false;
    bool _isTestStarted = false;
    int _topic_priority;
    int _msgCount;
    int _priority; //def not stated
    int _cpu_index; //def not stated
    int _msInterval;
    int _msgSizeMin;
    int _msgSizeMax;
    int _step;
    int _msgs_before_step;
    int _msgSize;
    int _last_rec_msg_id = -1;
    int _cur_size;
    std::mutex mu;
};
