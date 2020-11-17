#ifndef TESTPINGPONG_H_
#define TESTPINGPONG_H_

#include <zmq_addon.hpp>

#include "../../interface/ping_pong_interface.hpp"

class TestPingPong : public TestMiddlewarePingPong<zmq::message_t>
{
    public:

        TestPingPong(
            std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst);

        TestPingPong(
            std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, 
	    int msInterval, int msgSizeMin, int msgSizeMax, int step, 
	    int before_step, bool isFirst);
	
	void init();

        virtual ~TestPingPong();

        bool receive();

	short get_id(zmq::message_t &msg);

	unsigned long get_timestamp(zmq::message_t &msg);
    
	void publish(short id, unsigned size);
    private:
	int mcount;
	zmq::context_t pcontext;
	zmq::context_t scontext;
        zmq::socket_t psock;
        zmq::socket_t ssock;
};

#endif /* TESTPINGPONG_H_ */
