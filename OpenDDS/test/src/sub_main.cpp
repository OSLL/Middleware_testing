/*
 *
 *
 * Distributed under the OpenDDS License.
 * See: http://www.opendds.org/license.html
 */

#include <ace/Log_Msg.h>
#include <argparse/argparse.hpp>
#include "Subscriber.cpp"


int main(int argc, char* argv[]) {

    argparse::ArgumentParser parser("OpenDDS publisher argparsing");


    parser.add_argument("-t", "--topic")
            .help("-t --topic is required arugment with topic name");

    parser.add_argument("-f", "--filename")
            .help("-f --filename is required arugment with file name");

    parser.add_argument("-n", "--num_msg")
            .help("-n --num_msg is required arugment with msg count in step")
            .action([](const std::string& value) { return std::stoi(value); });

    parser.add_argument("-p", "--priority")
            .help("-p --priority is required arugment with priority")
            .action([](const std::string& value) { return std::stoi(value); });

    parser.add_argument("-c", "--cpu_index ")
            .help("-c --cpu_index is required arugment with cpu index")
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


    auto topic = parser.get<std::string>("-t");
    auto msg_count = parser.get<int>("-n");
    auto priority = parser.get<int>("-p");
    auto cpu_index = parser.get<int>("-c");
    auto filename = parser.get<std::string>("-f");

    Subscriber sub(topic, msg_count, priority, cpu_index, filename, 0);

    sub.createSubscriber(argc, argv);

    sub.StartTest();

    sub.cleanUp();

    return 0;
}
