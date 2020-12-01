#include "../../include/PingPong.h"


TestPingPongNode::TestPingPongNode( int argc, ACE_TCHAR *argv[],
        std::string &topic1, std::string &topic2, int msgCount, int prior, int cpu_index,
        std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst ):
        TestMiddlewarePingPong<Messenger::Message>(
                topic1, topic2, msgCount, prior, cpu_index,filename,
                topic_priority, msInterval, msgSize, isFirst )
    {



}

bool TestPingPongNode::receive() {

    return true;

}

short TestPingPongNode::get_id(Messenger::Message &msg) {
    return 0;
}

unsigned long TestPingPongNode::get_timestamp(Messenger::Message &msg) {
    return 0;
}

void TestPingPongNode::publish(short id, unsigned size) {
}
