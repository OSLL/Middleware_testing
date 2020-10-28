#ifndef TESTSUBSCRIBER_H_
#define TESTSUBSCRIBER_H_

#include <zmq_addon.hpp>

#include "../../interface/sub_interface.hpp"

class TestSubscriber : public TestMiddlewareSub<zmq::message_t>
{
    public:

        TestSubscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_prior, int max_msg_size);

        virtual ~TestSubscriber();

        bool receive();

	short get_id(zmq::message_t &msg);

	unsigned long get_timestamp(zmq::message_t &msg);
    private:
	int mcount;
	zmq::context_t context;
        zmq::socket_t sock;
};

#endif /* TESTSUBSCRIBER_H_ */
