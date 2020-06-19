#pragma once
#include <open62541/client_config_default.h>
#include <open62541/client_highlevel.h>
#include <open62541/plugin/log_stdout.h>
#include "DataType.hpp"
#include <cstdlib>
#include "../../interface/pub_interface.hpp"

class TestPublisher: public TestMiddlewarePub{
public:
    TestPublisher(std::string &topic,  int msgCount, int prior, int cpu_index,
                  int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step,
                  std::string &filename, int topic_priority):
                  TestMiddlewarePub(topic, msgCount, prior, cpu_index, min_msg_size, max_msg_size,
                          step, interval, msgs_before_step, filename, topic_priority)
                          {
        if(interval < 180)
            _msInterval = 180;
        types[0] = DataType;
        client = UA_Client_new();
        cc = UA_Client_getConfig(client);
        UA_ClientConfig_setDefault(cc);
        cc->customDataTypes = &customDataTypes;

        auto retval = UA_Client_connect(client, "opc.tcp://localhost:4840");
        if(retval != UA_STATUSCODE_GOOD) {
            UA_Client_delete(client);
            throw test_exception("Connection error!", MIDDLEWARE_ERROR);
        }
        UA_Variant_init(&value);

    }

    ~TestPublisher(){
        UA_Client_disconnect(client);
        UA_Client_delete(client);
    }

    unsigned long publish(short id, unsigned size) override {
        char topic_name[100];
        strcpy(topic_name, _topic_name.c_str());
        nodeId = UA_NODEID_STRING(1, topic_name);

        std::string data(size, 'a');
        char msg_data[size];
        strcpy(msg_data, data.c_str());
        auto str = UA_STRING(msg_data);

        UA_Variant *myVariant = UA_Variant_new();
        unsigned long long  timestamp = std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count();
        TestData msg = {str, id, timestamp};

        UA_Variant_setScalarCopy(myVariant, &msg, &DataType);

        timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::
                     now().time_since_epoch()).count();
        UA_Client_writeValueAttribute(client, nodeId, myVariant);
        unsigned long long time = std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count() - timestamp;

        UA_Variant_delete(myVariant);
        return time;
    }

protected:
    UA_ClientConfig *cc;
    UA_Client *client;
    UA_DataType types[1];
    UA_DataTypeArray customDataTypes = {NULL, 1, types};
    UA_Variant value;
    UA_NodeId nodeId;

};


/*int main(int argc, char *argv[]) {
    std::string topic("test_topic");
    std::string res("res.txt");
    TestPublisher pub(topic, 15, -1, -1, 50, 50, 0, 100,
            100, res, 0);
    try {
        pub.StartTest();
    }
    catch (test_exception e) {
        std::cout<<e.what();
    }
    return 0;
}*/
