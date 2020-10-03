#pragma once
#include <open62541/server.h>
#include <open62541/server_config_default.h>
extern "C" {
    #include "DataType.h"
}
#include "../../interface/pub_interface.hpp"
#include <stdlib.h>


static volatile UA_Boolean running = true;


int main() {
    UA_Server *server = UA_Server_new();
    auto config = UA_Server_getConfig(server);
    UA_ServerConfig_setDefault(config);
    config->publishingIntervalLimits.min = 6;
    config->maxNotificationsPerPublish = 30;
    config->samplingIntervalLimits.min = 6;

    addTestDataType(server, "test_topic");
    addTestDataVariableType(server, "test_topic");
    addTestDataVariable(server, "test_topic");
    addTestDataType(server, "test_topic1");
    addTestDataVariableType(server, "test_topic1");
    addTestDataVariable(server, "test_topic1");
    UA_StatusCode retval = UA_Server_run(server, &running);


    UA_Server_delete(server);
    return retval == UA_STATUSCODE_GOOD ? EXIT_SUCCESS : EXIT_FAILURE;
}
