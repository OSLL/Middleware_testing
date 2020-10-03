#include <open62541/server.h>
#include "DataType.h"

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