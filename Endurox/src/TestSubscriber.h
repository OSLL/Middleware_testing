#ifndef TESTSUBSCRIBER_H_
#define TESTSUBSCRIBER_H_

#include <atmi.h>

#include "../../interface/sub_interface.hpp"

class TestSubscriber : public TestMiddlewareSub<char*>
{
    public:

        TestSubscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_prior, int max_msg_size);

        virtual ~TestSubscriber();

        bool receive();

	short get_id(char* &msg);

	unsigned long get_timestamp(char* &msg);

	void msgHandler(char* data, long len, long flags);

    private:
	int n_received;
};

#endif /* TESTSUBSCRIBER_H_ */
