#include <zmq_addon.hpp>
#include <iostream>

#define MSGS_COUNT 5000
class Test{
private:
    zmq::context_t context;
    zmq::socket_t sock;
    zmq::message_t msg;
    int count;
public:
    Test(int msg_len): sock(context, zmq::socket_type::pub), msg(msg_len), count(0)
    {
        sock.bind("tcp://*:5555");
        //int immed = 1;
        //sock.setsockopt(ZMQ_IMMEDIATE, &immed, sizeof(immed));
        std::stringstream s;
        s << std::string(msg_len, 'a');
        auto str = s.str();
        memcpy(msg.data(), str.c_str(), msg_len);
    }

    int start_test(){
        while (count < MSGS_COUNT) {
            if (sock.send(msg, zmq::send_flags::none)){
                count++;
                std::cout << count << std::endl;
            } else
                return 1;
        }
        return 0;
    }
};

int main(int argc, char **argv)
{
    Test test(60);
    if(test.start_test())
        return 1;
    return 0;
}
