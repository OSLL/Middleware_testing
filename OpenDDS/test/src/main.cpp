/*
 *
 *
 * Distributed under the OpenDDS License.
 * See: http://www.opendds.org/license.html
 */

#include <ace/Log_Msg.h>
#include <argparse/argparse.hpp>
#include "Publisher.cpp"


int main(int argc, char* argv[]) {

    argparse::ArgumentParser parser("OpenDDS publisher argparsing");
    parser.add_argument("-t", "--topic")
            .help("-t --topic is required arugment with topic name");

    parser.add_argument("-n", "--num_msg")
            .help("-n --num_msg is required arugment with msg count in step")
            .action([](const std::string& value) { return std::stoi(value); });

    parser.add_argument("-i", "--interval")
            .help("-i --interval is required arugment with msg interval in ms")
            .action([](const std::string& value) { return std::stoi(value); });

    parser.add_argument("-p", "--priority")
            .help("-p --priority is required arugment with priority")
            .action([](const std::string& value) { return std::stoi(value); });

    parser.add_argument("-c", "--cpu_index ")
            .help("-c --cpu_index is required arugment with cpu index")
            .action([](const std::string& value) { return std::stoi(value); });

    parser.add_argument("--min_byte")
            .help("--min_byte is required arugment with min size msg in bytes")
            .action([](const std::string& value) { return std::stoi(value); });

    parser.add_argument("--max_byte")
            .help("--max_byte is required arugment with max size msg in bytes")
            .action([](const std::string& value) { return std::stoi(value); });

    parser.add_argument("-s", "--step")
            .help("-s --step is required arugment with step count")
            .action([](const std::string& value) { return std::stoi(value); });

    parser.add_argument("--msg_before_step")
            .help("msg_before_step is required arugment with msg count before step")
            .action([](const std::string& value) { return std::stoi(value); });

    parser.add_argument("-DCPSConfigFile");
    parser.add_argument("-ORBDebugLevel");

    try {
        parser.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cout << err.what() << std::endl;
        std::cout << parser;
        exit(0);
    }


    auto interval = parser.get<int>("-i");
    auto msg_count = parser.get<int>("-n");
    auto priority = parser.get<int>("-p");
    auto cpu_index = parser.get<int>("-c");
    auto min_bytes = parser.get<int>("--min_byte");
    auto max_bytes = parser.get<int>("--max_byte");
    auto step = parser.get<int>("-s");
    auto msg_count_before_step = parser.get<int>("--msg_before_step");
    auto topic = parser.get<std::string>("-t");

    auto topics = std::vector<std::string>{topic};

    Publisher pub(topics, msg_count, priority, cpu_index, min_bytes, max_bytes, step,
                  interval, msg_count_before_step);

    pub.createPublisher(argc, argv);

    pub.StartTest();

    return 0;
}
