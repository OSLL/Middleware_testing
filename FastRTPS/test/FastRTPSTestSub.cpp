#include "FastRTPSTestSub.h"

TestSubscriber* FastRTPSTestSub::createSubscriber(std::string topic) {
    TestSubscriber* sub = new TestSubscriber();
    sub->init(topic, _msgCount, _byteSizeMax);
    return sub;
}

std::tuple<std::vector<std::string>, std::vector<unsigned long>> FastRTPSTestSub::receive() {
    return _subscriber->receive();
}

