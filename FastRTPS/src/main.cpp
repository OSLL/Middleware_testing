#include <argparse/argparse.hpp>
#include <nlohmann/json.hpp>

#include <fastrtps/Domain.h>

#include "TestPublisher.h"
#include "TestSubscriber.h"
#include "TestPingPongNode.h"

using namespace eprosima;
using namespace fastrtps;
using namespace rtps;

int main(int argc, char** argv)
{
    argparse::ArgumentParser parser("FastRTPS node argparsing");
    parser.add_argument("-c", "--config")
            .required()
            .help("-c --conf is required argument with config path");

    parser.add_argument("-t", "--type")
            .required()
            .help("-t --type is required argument with type of node");

    parser.add_argument("--first")
            .implicit_value(true)
            .default_value(false)
            .help("--first is required argument if ping_pong type is specified with type of node");

    try {
        parser.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cout << err.what() << std::endl;
        std::cout << parser;
        exit(1);
    }

    std::string config_filename = parser.get<std::string>("-c");
    std::string type_name = parser.get<std::string>("-t");

    bool isFirst = parser.get<bool>("--first");

        nlohmann::json args;
    std::ifstream file(config_filename);
    if(!file.is_open()) {
        std::cout << "Cannot open file " << config_filename << std::endl;
        return 0;
    }
    file >> args;
    file.close();

    std::string topic1 = args["topic"][0];
    std::string topic2 = args["topic"][1];
    std::string filename_pub = args["res_filenames"][0];
    std::string filename_sub = args["res_filenames"][1];
    int m_count = args["m_count"];
    int priority_pub = args["priority"][0];
    int priority_sub = args["priority"][1];
    int cpu_index_pub = args["cpu_index"][0];
    int cpu_index_sub = args["cpu_index"][1];
    int min_msg_size = args["min_msg_size"];
    int max_msg_size = args["max_msg_size"];
    int step = args["step"];
    int msgs_before_step = args["msgs_before_step"];
    int interval = args["interval"];
    int topic_prior = args["topic_priority"];

    try {
        if (type_name == "publisher"){
            TestPublisher publisher(topic1, m_count, priority_pub, cpu_index_pub, min_msg_size, max_msg_size, step, interval,
                                     msgs_before_step, filename_pub, topic_prior);
            publisher.StartTest();
        }
	else if (type_name == "subscriber") {
            TestSubscriber subscriber(topic1, m_count, priority_sub, cpu_index_sub, filename_sub, topic_prior, max_msg_size);
            subscriber.StartTest();
        }
	else if (type_name == "ping_pong"){
            std::string filename;
            if(isFirst)
                filename = filename_pub;
            else
                filename = filename_sub;
            if (interval == 0) {
                TestPingPongNode ping_pong(topic1, topic2, m_count, priority_pub, cpu_index_pub, filename,
                                           topic_prior, interval, min_msg_size, isFirst);
                ping_pong.StartTest();
            }
            else{
                TestPingPongNode ping_pong(topic1, topic2, m_count, priority_pub,
                                           cpu_index_pub, filename, topic_prior, interval, min_msg_size,
                                           max_msg_size, step, msgs_before_step, isFirst);
                ping_pong.StartTest();
            }
        }
        else{
            std::cout << "Wrong node type specified!" << std::endl;
            return 0;
        }
    }
    catch (test_exception& e){
        std::cout<< e.what() << std::endl;
        return -e.get_ret_code();
    }
    catch (std::exception& e){
        std::cout<< e.what()<< std::endl;
        return -MIDDLEWARE_ERROR;
    }

    /*int type = 0;
    
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
    }*/
    Domain::stopAll();
    return 0;
}
