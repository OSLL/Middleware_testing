#include "ping_pong.h"


TestPingPongNode::TestPingPongNode( int argc, ACE_TCHAR *argv[],
        std::string &topic1, std::string &topic2, int msgCount, int prior, int cpu_index,
        std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst ):
        TestMiddlewarePingPong<Messenger::Message>(
                topic1, topic2, msgCount, prior, cpu_index,filename,
                topic_priority, msInterval, msgSize, isFirst )
    {
    if (!_isFirst)
        topic1.swap(topic2);

    pub = Publisher(topic1, msgCount, prior, cpu_index, 1, 100,
                    1, msInterval, 1, filename, topic_priority);
    pub.createPublisher(argc, argv);

    sub = Subscriber(topic2, msgCount, prior, cpu_index, filename, topic_priority);
    sub.createSubscriber(argc, argv);


}

bool TestPingPongNode::receive() {
    Messenger::Message msg;
    bool result =  sub.receive(msg);

    if (result)
        write_received_msg(msg);

    return result;

}

short TestPingPongNode::get_id(Messenger::Message &msg) {
    return sub.get_id(msg);
}

unsigned long TestPingPongNode::get_timestamp(Messenger::Message &msg) {
    return sub.get_timestamp(msg);
}

void TestPingPongNode::publish(short id, unsigned size) {
    if (_isFirst)
        pub.publish(id, size, 'b');
    else
        pub.publish(id, size, 'a');
}
