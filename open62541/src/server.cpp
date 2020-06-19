#pragma once
#include <open62541/server.h>
#include <open62541/server_config_default.h>
#include "DataType.hpp"
#include "../../interface/pub_interface.hpp"
#include <signal.h>
#include <stdlib.h>


static const UA_NodeId testDataVariableTypeId = {1, UA_NODEIDTYPE_NUMERIC, {4243}};
static volatile UA_Boolean running = true;

void addTestDataType(UA_Server *server, char *name)
{
    UA_DataTypeAttributes attr = UA_DataTypeAttributes_default;
    attr.displayName = UA_LOCALIZEDTEXT("en-US", "Test Data Type");

    UA_Server_addDataTypeNode(
            server, DataType.typeId, UA_NODEID_NUMERIC(0, UA_NS0ID_STRUCTURE),
            UA_NODEID_NUMERIC(0, UA_NS0ID_HASSUBTYPE), UA_QUALIFIEDNAME(1,
                                                                        name), attr, NULL, NULL);
}

void addTestDataVariableType(UA_Server *server, char *name) {
    UA_VariableTypeAttributes dattr = UA_VariableTypeAttributes_default;
    dattr.description = UA_LOCALIZEDTEXT("en-US", name);
    dattr.displayName = UA_LOCALIZEDTEXT("en-US", name);
    dattr.dataType = DataType.typeId;
    dattr.valueRank = UA_VALUERANK_SCALAR;

    TestData data;
    data.id = 0;
    data.timestamp = 0;
    data.data = UA_STRING("data");
    UA_Variant_setScalar(&dattr.value, &data, &DataType);

    UA_Server_addVariableTypeNode(server, testDataVariableTypeId,
                                  UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                                  UA_NODEID_NUMERIC(0, UA_NS0ID_HASSUBTYPE),
                                  UA_QUALIFIEDNAME(1, name),
                                  UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE),
                                  dattr, NULL, NULL);

}

void addTestDataVariable(UA_Server *server, char *name) {
    TestData data;
    data.id = 0;
    data.timestamp = 0;
    data.data = UA_STRING("data");
    UA_VariableAttributes vattr = UA_VariableAttributes_default;
    vattr.description = UA_LOCALIZEDTEXT("en-US", name);
    vattr.displayName = UA_LOCALIZEDTEXT("en-US", name);
    vattr.dataType = DataType.typeId;
    vattr.valueRank = UA_VALUERANK_SCALAR;
    vattr.minimumSamplingInterval = 0;
    vattr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;
    UA_Variant_setScalar(&vattr.value, &data, &DataType);

    UA_Server_addVariableNode(server, UA_NODEID_STRING(1, name),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER),
                              UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES),
                              UA_QUALIFIEDNAME(1, name),
                              testDataVariableTypeId, vattr, NULL, NULL);
}

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
