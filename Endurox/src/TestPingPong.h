#ifndef TESTPINGPONG_H_
#define TESTPINGPONG_H_

#include <atmi.h>

#include "../../interface/ping_pong_interface.hpp"

class TestPingPong : public TestMiddlewarePingPong<char*>
{
    public:

        TestPingPong(
            std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst);

        virtual ~TestPingPong();

        bool receive();

	short get_id(char* &msg);

	unsigned long get_timestamp(char* &msg);
    
	void publish(short id, unsigned size);

	void msgHandler(char* data, long len, long flags);
    
    private:
	int n_received;

	int n_before;

	char* strbuf;

	TPQCTL wqctl;
	
	TPQCTL rqctl;
};

#endif /* TESTPINGPONG_H_ */
