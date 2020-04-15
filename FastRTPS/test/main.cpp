#include "TestPublisher.h"
#include "TestSubscriber.h"
#include "../../nlohmann/json.hpp"

#include <fastrtps/Domain.h>

using namespace eprosima;
using namespace fastrtps;
using namespace rtps;

int main(int argc, char** argv)
{
    int type = 0;
    
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

    if (type==0 || argc < 3)
    {
        std::cout << "Error: Incorrect arguments." << std::endl;
        std::cout << "Usage: " << std::endl << std::endl;
        std::cout << argv[0] << " publisher|subscriber params.json" << std::endl << std::endl;
        return 0;
    }

    nlohmann::json args;
    std::ifstream file(argv[2]);
    if(!file.is_open()) {
        std::cout << "Cannot open file " << argv[2] << std::endl;
        return 0;
    }
    file >> args;
    file.close();

    std::string topic = args["topic"];
    std::string filename = args["res_filenames"][type-1];
    int m_count = args["m_count"];
    int priority = args["priority"][type-1];
    int cpu_index = args["cpu_index"][type-1];
    int min_msg_size = args["min_msg_size"];
    int max_msg_size = args["max_msg_size"];
    int step = args["step"];
    int msgs_before_step = args["msgs_before_step"];
    int interval = args["interval"];
    int topic_prior = args["topic_priority"];

    try{
        switch(type)
	{
            case 1:
            {
                TestPublisher mypub(topic, m_count, priority, cpu_index, min_msg_size, max_msg_size, step, interval, msgs_before_step, filename, topic_prior);
                mypub.StartTest();
                break;
            }
	    case 2:
            {
                TestSubscriber mysub(topic, m_count, priority, cpu_index, filename, topic_prior, max_msg_size);
                mysub.StartTest();
                break;
            }
        }
    }
    catch (test_exception& e){
        std::cout << e.what() << std::endl;
        return -e.get_ret_code();
    }
    catch (std::exception& e){
        std::cout << e.what() << std::endl;
        return -MIDDLEWARE_ERROR;
    }
    Domain::stopAll();
    return 0;
}
