#include "test_interface.hpp"
#include "TestSubscriber.h"

class FastRTPSTestSub : public TestMiddlewareSub<TestSubscriber>
{
public:

    explicit FastRTPSTestSub(std::string topic, int msgCount=0, int prior = -1, int cpu_index = -1, int max_msg_size=64000) :
    TestMiddlewareSub(topic, msgCount, prior, cpu_index, max_msg_size) 
    {}

    virtual TestSubscriber* createSubscriber(std::string topic);

    virtual std::vector<std::string> receive();  //возвращает вектор принятых сообщений

//    virtual void setQoS(std::string filename);
};

