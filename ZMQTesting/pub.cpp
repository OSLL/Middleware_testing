#define ZMQ_CPP14
#include <zmq_addon.hpp>
#include <fstream>
#include <iostream>
#include <chrono>
#include <ctime>
#include <vector>
#include <unistd.h>

#define MSGS_COUNT 5000
using namespace std::chrono;
class Test{
private:
    zmq::context_t context;
    zmq::socket_t sock;
    zmq::message_t msg;
    std::vector<unsigned long int> time_list;
    sched_param priority;
    int count;
public:
    Test(int msg_len): sock(context, zmq::socket_type::pub), msg(msg_len), time_list(MSGS_COUNT), count(0)
    {
        sock.bind("tcp://127.0.0.1:5600");
        int max_q = 6000;
        sock.setsockopt(ZMQ_SNDHWM, &max_q, sizeof(max_q));
        int dont_drop = 1;
        sock.setsockopt(ZMQ_XPUB_NODROP, &dont_drop, sizeof(dont_drop));
	    usleep(4000000);
	    pid_t id = getpid();
        std::ofstream f_task("/sys/fs/cgroup/cpuset/pub_cpuset/tasks", std::ios_base::out);
        if(!f_task.is_open()){
           std::cout << "Error in adding to cpuset" << std::endl;
        }
        else{
          auto s = std::to_string(id);
          f_task.write(s.c_str(),s.length());
        }
        f_task.close();
        priority.sched_priority = sched_get_priority_max(SCHED_FIFO);
        int err = sched_setscheduler(id, SCHED_FIFO, &priority);
        if(err)
          std::cout << "Error in setting priority: " << -err << std::endl;
        std::stringstream s;
        s << std::string(msg_len, 'a');
        auto str = s.str();
        memcpy(msg.data(), str.c_str(), msg_len);
    }

    int start_test(){
        while (count < MSGS_COUNT) {
            if (sock.send(msg, zmq::send_flags::none)){
		        time_list[count] = duration_cast<nanoseconds>(high_resolution_clock::now().time_since_epoch()).count();
                count++;
		        int i = 0;
            } else
                return 1;
        }
        return 0;
    }
    ~Test() {
        std::ofstream file("publisher.txt");
        for(unsigned i=0; i<time_list.size();i++)
            file << time_list[i] << ' ';
        file << std::endl;
        file.close();
    }
};

int main(int argc, char **argv)
{
    if(argc < 2){
        std::cout<< "Error: too few arguments!"<< std::endl;
        return 0;
    }
    int len;
    try {
        std::string str(argv[1]);
        len = std::stoi(str);
    }
    catch (std::invalid_argument const &e){
        std::cout << "Error: bad argument!" <<std::endl;
        return 0;
    }
    catch (std::out_of_range const &e){
        std::cout << "Error: integer overflow!" <<std::endl;
        return 0;
    }
    Test test(len);
    if(test.start_test())
        return 1;
    return 0;
}
