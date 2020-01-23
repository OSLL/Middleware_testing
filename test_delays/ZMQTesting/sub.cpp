#include <zmq_addon.hpp>
#include <iostream>
#include <cerrno>

#define MSGS_COUNT 5000
class Test{
private:
    zmq::context_t context;
    zmq::socket_t sock;
    zmq::message_t msg;
    int count;
public:
    Test(int msg_len): sock(context, zmq::socket_type::sub), msg(msg_len), count(0)
    {
        sock.connect("tcp://127.0.0.1:5555");
        sock.setsockopt(ZMQ_SUBSCRIBE, "", 0);
        sock.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    }

    int start_test(){
        while (count < MSGS_COUNT) {
            if (sock.recv(msg, zmq::recv_flags::none)){
                count++;
                std::cout << count << std::endl;
            }
            else{
                if(errno != EAGAIN)
                    std::cout<< std::strerror(errno)<<std::endl;
            }
        }
        return 0;
    }
};

int main(int argc, char **argv)
{
    Test test(600);
    if(test.start_test())
        return 1;
    return 0;
}
