#include <argparse/argparse.hpp>
#include <Subscriber.h>


int main(int argc, char* argv[]) {

    //*argv = const_cast<char *>(std::string{"-DCPSConfigFile shmem.ini"}.c_str());

    argparse::ArgumentParser parser("OpenDDS subscriber argparsing");
    parser.add_argument("-c", "--config")
            .required()
            .help("-c --conf is required arugment with config path");

    parser.add_argument("-DCPSConfigFile");

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
        std::string filename = args["res_filenames"][1];
        int message_count = args["m_count"];
        int priority = args["priority"][1];
        int cpu_index = args["cpu_index"][1];

        try {

            Subscriber sub(topic, message_count, priority, cpu_index, filename, 0);
            sub.createSubscriber(argc, argv);
            sub.StartTest();
            sub.cleanUp();

        } catch (test_exception& e) {

            std::cout<< e.what() << std::endl;
            return -e.get_ret_code();

        } catch (std::exception& e){

            std::cout<< e.what()<< std::endl;
            return -MIDDLEWARE_ERROR;
        }

    return 0;
}
