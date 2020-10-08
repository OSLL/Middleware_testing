#include <open62541/plugin/log_stdout.h>
#include <open62541/server_config_default.h>
#include <string>
#include <chrono>

extern "C" {
    #include "../DataType.h"
    #include "pubsub.h"
}


#define TOPIC_1 (char *)"test_topic"

UA_Boolean running = true;
static void stopHandler(int sign) {
    UA_LOG_INFO(UA_Log_Stdout, UA_LOGCATEGORY_SERVER, "received ctrl-c");
    running = false;
}
unsigned long publish(UA_Server *server, short id, unsigned size){
    UA_Variant value;
    TestData msg;
    char data[size];
    memcpy(data, std::string(size, 'a').c_str(), size);
    auto ts = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::
            high_resolution_clock::now().time_since_epoch()).count();
    msg = {
            .data = UA_STRING(data),
            .id = id,
            .timestamp = static_cast<UA_UInt64>(ts)
            };

    UA_Variant_setScalar(&value, &msg, &DataType);
    UA_Server_run_iterate(server, true);
    UA_Server_writeValue(server, testDataVariableTypeId, value);
    return 0;
}

static int run(UA_String *transportProfile,
               UA_NetworkAddressUrlDataType *networkAddressUrl) {
    signal(SIGINT, stopHandler);
    signal(SIGTERM, stopHandler);

    UA_Server *server = UA_Server_new();
    UA_ServerConfig *config = UA_Server_getConfig(server);
    UA_ServerConfig_setDefault(config);

    config->pubsubTransportLayers =
            (UA_PubSubTransportLayer *) UA_calloc(2, sizeof(UA_PubSubTransportLayer));
    if(!config->pubsubTransportLayers) {
        UA_Server_delete(server);
        return EXIT_FAILURE;
    }
    config->pubsubTransportLayers[0] = UA_PubSubTransportLayerUDPMP();
    config->pubsubTransportLayersSize++;

    addTestDataType(server, TOPIC_1);
    addTestDataVariableType(server, TOPIC_1);
    addTestDataVariable(server, TOPIC_1);
    addPubSubConnection(server, transportProfile, networkAddressUrl);
    addPublishedDataSet(server);
    addDataSetField(server);
    addWriterGroup(server);
    addDataSetWriter(server);

    UA_StatusCode retval = UA_Server_run_startup(server);

    if(retval != UA_STATUSCODE_GOOD){
        UA_Server_delete(server);
        return retval;
    }
    int i = 0;
    while (running){
        if (i < 100)
            publish(server, i, 14);
        usleep(50000);
        i++;
        UA_Variant value;
        UA_Server_readValue(server, testDataVariableTypeId, &value);

        auto msg = (TestData *)value.data;
        if (msg != NULL)
            printf("%d %lu\n", msg->id, msg->timestamp);
    }
    retval = UA_Server_run_shutdown(server);
    UA_Server_delete(server);
    return retval == UA_STATUSCODE_GOOD ? EXIT_SUCCESS : EXIT_FAILURE;
}




int main() {
    UA_String transportProfile =
            UA_STRING((char *)"http://opcfoundation.org/UA-Profile/Transport/pubsub-udp-uadp");
    UA_NetworkAddressUrlDataType networkAddressUrl =
            {UA_STRING_NULL , UA_STRING((char *)"opc.udp://224.0.0.22:4840/")};

    return run(&transportProfile, &networkAddressUrl);
}