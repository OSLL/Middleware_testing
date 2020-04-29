#include <argparse/argparse.hpp>
#include <Publisher.h>



int main(int argc, char* argv[]) {

    freopen("cyclone_pub_output.txt","w",stdout);

    argparse::ArgumentParser parser("CycloneDDS publisher argparsing");
    parser.add_argument("-c", "--config")
            .required()
            .help("-c --conf is required arugment with config path");

    try {
        parser.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cout << err.what() << std::endl;
        std::cout << parser;
        exit(1);
    }

    auto config_path = parser.get<std::string>("-c");

    nlohmann::json args;
    std::ifstream config_file(config_path);

    if(!config_file.is_open()) {
        std::cout << "Cannot open config_file " << config_path << std::endl;
        return 1;
    }

    config_file >> args;
    config_file.close();

    std::string topic = args["topic"];
    std::string filename = args["res_filenames"][0];
    int m_count = args["m_count"];
    int priority = args["priority"][0];
    int cpu_index = args["cpu_index"][0];
    int min_msg_size = args["min_msg_size"];
    int max_msg_size = args["max_msg_size"];
    int step = args["step"];
    int msgs_before_step = args["msgs_before_step"];
    int interval = args["interval"];
    int topic_prior = args["topic_priority"];

    try {

        Publisher<Messenger_Message> publisher(topic, m_count, priority, cpu_index, min_msg_size, max_msg_size,
                            step, interval, msgs_before_step, filename, topic_prior );
        publisher.create(Messenger_Message_desc);
        publisher.StartTest();

    }
    catch (test_exception& e){

        std::cout<< e.what() << std::endl;
        return -e.get_ret_code();

    }
    catch (std::exception& e){

        std::cout<< e.what()<< std::endl;
        return -MIDDLEWARE_ERROR;
    }

    return 0;
}
