#pragma once
#include <open62541/client_config_default.h>
#include <open62541/client_highlevel.h>
#include <open62541/client_subscriptions.h>
#include <open62541/plugin/log_stdout.h>
#include "DataType.hpp"
#include "../../interface/ping_pong_interface.hpp"

class TestPingPongNode: public TestMiddlewarePingPong<TestData>{
public:
    TestPingPongNode(
            std::string &topic1, std::string &topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst
    ): TestMiddlewarePingPong<TestData>(topic1, topic2, msgCount, prior, cpu_index, filename, topic_priority, msInterval, msgSize, isFirst)
    {
        strcpy(topic_name1, _topic_name1.c_str());
        strcpy(topic_name2, _topic_name2.c_str());
        if (!_isFirst){
            strcpy(topic_name1, _topic_name2.c_str());
            strcpy(topic_name2, _topic_name1.c_str());
        }


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
        UA_CreateSubscriptionRequest request = UA_CreateSubscriptionRequest_default();
        request.requestedPublishingInterval = 6.0;
        UA_CreateSubscriptionResponse response = UA_Client_Subscriptions_create(client, request,
                                                                                this, NULL, NULL);

        subId = response.subscriptionId;
        if(response.responseHeader.serviceResult != UA_STATUSCODE_GOOD)
            throw test_exception("Error with creating subscription!", MIDDLEWARE_ERROR);

        isRunning = false;
        UA_MonitoredItemCreateRequest monRequest =
                UA_MonitoredItemCreateRequest_default(UA_NODEID_STRING(1, topic_name2));
        UA_MonitoredItemCreateResult monResponse =
                UA_Client_MonitoredItems_createDataChange(client, response.subscriptionId,
                                                          UA_TIMESTAMPSTORETURN_BOTH,
                                                          monRequest, &isRunning, handler_ValChanged, NULL);
        if(monResponse.statusCode != UA_STATUSCODE_GOOD)
            throw test_exception("Monitoring item error!", MIDDLEWARE_ERROR);


    }

    bool receive() override {
        isReceived = false;
        isRunning = true;
        UA_Client_run_iterate(client, 1000);
        return isReceived;
    }
    short get_id(TestData &msg) override{
        return msg.id;
    }
    unsigned long get_timestamp(TestData &msg) override {
        return msg.timestamp;
    }

    void publish(short id, unsigned size) override {
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

        UA_Client_writeValueAttribute(client, nodeId, myVariant);

        UA_Variant_delete(myVariant);
    }

protected:
    UA_NodeId nodeId;
    UA_ClientConfig *cc;
    UA_Client *client;
    UA_DataType types[1];
    UA_DataTypeArray customDataTypes = {NULL, 1, types};
    UA_Variant value;
    char topic_name1[100];
    char topic_name2[100];
    bool isReceived;
    UA_UInt32 subId;
    static void handler_ValChanged(UA_Client *client, UA_UInt32 subId, void *subContext,
                                   UA_UInt32 monId, void *monContext, UA_DataValue *value) {
        if(!isRunning)
            return;
        char *topic = ((TestPingPongNode *)subContext)->topic_name2;
        TestData data;
        UA_Variant *val = UA_Variant_new();
        auto start_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::
                                                                                    now().time_since_epoch()).count();
        UA_StatusCode retval = UA_Client_readValueAttribute(client, UA_NODEID_STRING(1,
                                                                                     topic), val);
        unsigned long proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::
                                                                                       high_resolution_clock::now().time_since_epoch()).count() - start_timestamp;
        if(retval == UA_STATUSCODE_GOOD && UA_Variant_isScalar(val)) {
            data = *((TestData *)val->data);
            ((TestPingPongNode *)subContext)->write_received_msg(data);
            ((TestPingPongNode *)subContext)->isReceived = true;
            printf("the value is: %d %lu\n", data.id, data.timestamp);
        }
        UA_Variant_delete(val);
    }
};