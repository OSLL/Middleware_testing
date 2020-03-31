#include "TestPublisher.h"
#include "TestSubscriber.h"
#include "nlohmann/json.hpp"

#include <fastrtps/Domain.h>

using namespace eprosima;
using namespace fastrtps;
using namespace rtps;

int main(int argc, char** argv)
{
    int type = 0;
    std::vector<std::string> topics;
    std::vector<std::string> res_filenames;
    int m_count = 5000;
    int priority = -1;
    int cpu_index = -1;
    int min_msg_size = 0;
    int max_msg_size = 64000;
    int step = 0;
    int msgs_before_step = 100;
    int interval = 0;
    
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
	
    if(args["topics"] != nullptr){
        for(auto topic : args["topics"])
            topics.push_back(topic);
    }
    if(args["res_filenames"] != nullptr){
        for(auto res_filename : args["res_filenames"])
            res_filenames.push_back(res_filename);
    }
    if(args["m_count"] != nullptr){
        m_count = args["m_count"];
    }
    if(args["min_msg_size"] != nullptr){
        min_msg_size = args["min_msg_size"];
    }
    if(args["max_msg_size"] != nullptr){
        max_msg_size = args["max_msg_size"];
    }
    if(args["step"] != nullptr){
        step = args["step"];
    }
    if(args["msgs_before_step"] != nullptr){
        msgs_before_step = args["msgs_before_step"];
    }
    if(args["priority"] != nullptr){
        priority = args["priority"];
    }
    if(args["cpu_index"] != nullptr){
        cpu_index = args["cpu_index"];
    }
    if(args["interval"] != nullptr){
        interval = args["interval"];
    }


    switch(type)
    {
        case 1:
        {
            TestPublisher mypub(topics[0], m_count, priority, cpu_index, min_msg_size, max_msg_size, step, interval, msgs_before_step);
            mypub.StartTest();
	    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
            break;
        }
        case 2:
        {
            TestSubscriber mysub(topics, m_count, priority, cpu_index, res_filenames, max_msg_size);
            mysub.StartTest();
            break;
        }
    }
    Domain::stopAll();
    return 0;
}
