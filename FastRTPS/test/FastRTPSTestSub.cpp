#include "FastRTPSTestSub.h"

TestSubscriber* FastRTPSTestSub::createSubscriber(std::string topic) {
    TestSubscriber* sub = new TestSubscriber();
    sub->init(topic, _msgCount, _byteSizeMax);
    return sub;
}

std::vector<std::string> FastRTPSTestSub::receive() {
    return std::vector<std::string>();
}

