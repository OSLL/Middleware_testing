#include "FastRTPSTestPub.h"

TestPublisher* FastRTPSTestPub::createPublisher(std::string topic) {
    TestPublisher* pub = new TestPublisher();
    pub->init(topic, _byteSizeMax);
    return pub;
}

void FastRTPSTestPub::publish(std::string &msg) {
    _publisher->publish(msg);
}
