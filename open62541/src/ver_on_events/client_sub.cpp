#pragma once
#include <open62541/client_config_default.h>
#include <open62541/client_highlevel.h>
#include <open62541/client_subscriptions.h>
#include <open62541/plugin/log_stdout.h>
#include "../../../interface/sub_interface.hpp"
#include "../DataType.h"

static bool isRunning;

class TestSubscriber: public TestMiddlewareSub<TestData>{
public:
    TestSubscriber(std::string &topic, int msgCount, int prior,
                   int cpu_index, std::string &filename, int topic_priority):
                   TestMiddlewareSub<TestData>(topic, msgCount, prior, cpu_index,
                           filename, topic_priority),
                            isReceived(false){
        types[0] = DataType;
        strcpy(topic_name, _topic_name.c_str());
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
                UA_MonitoredItemCreateRequest_default(UA_NODEID_STRING(1, topic_name));
        UA_MonitoredItemCreateResult monResponse =
                UA_Client_MonitoredItems_createDataChange(client, response.subscriptionId,
                                                          UA_TIMESTAMPSTORETURN_BOTH,
                                                          monRequest, &isRunning, handler_ValChanged, NULL);
        if(monResponse.statusCode != UA_STATUSCODE_GOOD)
            throw test_exception("Monitoring item error!", MIDDLEWARE_ERROR);


    }
    ~TestSubscriber(){
        if(UA_Client_Subscriptions_deleteSingle(client, subId) == UA_STATUSCODE_GOOD)
            printf("Subscription removed\n");
        UA_Variant_clear(&value);
        UA_Client_disconnect(client);
        UA_Client_delete(client);
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

protected:
    UA_ClientConfig *cc;
    UA_Client *client;
    UA_DataType types[1];
    UA_DataTypeArray customDataTypes = {NULL, 1, types};
    UA_Variant value;
    char topic_name[100];
    bool isReceived;
    UA_UInt32 subId;
    static void handler_ValChanged(UA_Client *client, UA_UInt32 subId, void *subContext,
                             UA_UInt32 monId, void *monContext, UA_DataValue *value) {
        if(!isRunning)
            return;
        char *topic = ((TestSubscriber *)subContext)->topic_name;
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
            ((TestSubscriber *)subContext)->write_received_msg(data, proc_time);
            ((TestSubscriber *)subContext)->isReceived = true;
            //printf("the value is: %d %lu\n", data.id, data.timestamp);
        }
        UA_Variant_delete(val);
    }
};
