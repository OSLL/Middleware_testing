#include "TestPublisher.h"
#include "TestSubscriber.h"
#include "FastRTPSTestPub.h"
#include "FastRTPSTestSub.h"

#include <fastrtps/Domain.h>

using namespace eprosima;
using namespace fastrtps;
using namespace rtps;

int main(int argc, char** argv)
{
    int type = 0;
    int m_count = 5000;
    int priority = -1;
    int cpu_index = -1;
    int min_msg_size = 0;
    int max_msg_size = 64000;
    int step = 0;
    int interval = 0;
    int msgs_before_step = 100;

    if (argc > 1)
    {
        if (strcmp(argv[1], "publisher") == 0)
        {
            type = 1;
        }
        else if (strcmp(argv[1], "subscriber") == 0)
        {
            type = 2;
        }
    }
    if (argc > 2) {
        m_count = atoi(argv[2]);
    }
    if (argc > 3) {
        priority = atoi(argv[3]);
    }
    if (argc > 4) {
        cpu_index = atoi(argv[4]);
    }
    if(argc > 5) {
        min_msg_size = atoi(argv[5]);
    }
    if(argc > 6) {
        max_msg_size = atoi(argv[6]);
    }
    if(argc > 7) {
        step = atoi(argv[7]);
    }
    if(argc > 8) {
        interval = atoi(argv[8]);
    }
    if(argc > 9) {
        msgs_before_step = atoi(argv[9]);
    }
    if (type==0)
    {
        std::cout << "Error: Incorrect arguments." << std::endl;
        std::cout << "Usage: " << std::endl << std::endl;
        std::cout << argv[0] << " publisher|subscriber [m_count] [priority] [cpu_index] [min_msg_size] [max_msg_size] [step] [interval] [msgs_before_step]" << std::endl << std::endl;
        return 0;
    }
    std::cout << "Starting "<< std::endl;

    std::vector<std::string> msgs;

    switch(type)
    {
        case 1:
        {
            FastRTPSTestPub mypub("TestTopic", m_count, priority, cpu_index, min_msg_size, max_msg_size, step, interval, msgs_before_step);
            mypub.test();
            break;
        }
        case 2:
        {
            FastRTPSTestSub mysub("TestTopic", m_count, priority, cpu_index, max_msg_size);
            mysub.test();
            break;
        }
    }
    Domain::stopAll();
    return 0;
}
