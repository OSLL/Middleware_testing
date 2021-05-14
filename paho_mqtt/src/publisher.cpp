#include <chrono>
#include <iostream>
#include <unistd.h>
#include "../../interface/pub_interface.hpp"
#include "./data_type.h"

class TestPublisher: public TestMiddlewarePub{
public:
    TestPublisher(std::string topic,  int msgCount, int prior, int cpu_index,
                  int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step, std::string &filename,
                  int topic_priority):
            TestMiddlewarePub(topic, msgCount, prior, cpu_index, min_msg_size, max_msg_size, step, interval,
                              msgs_before_step, filename, topic_priority)
    {
        MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

        int rc;

        if ((rc = MQTTClient_create(&_client, ADDRESS, filename.c_str(),
                                    MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTCLIENT_SUCCESS)
        {
            printf("Failed to create client, return code %d\n", rc);
            exit(EXIT_FAILURE);
        }

        conn_opts.keepAliveInterval = 20;
        conn_opts.cleansession = 1;
        if ((rc = MQTTClient_connect(_client, &conn_opts)) != MQTTCLIENT_SUCCESS)
        {
            printf("Failed to connect, return code %d\n", rc);
            exit(EXIT_FAILURE);
        }

        _pubmsg.qos = QOS;
        _pubmsg.retained = 0;

    }


    unsigned long publish(short id, unsigned size) override {
        int rc;
        std::string data(size, 'a');
        _pubmsg.payload = (void *) data.c_str();
        _pubmsg.payloadlen = (int)strlen(data.c_str());
        unsigned long proc_time = 0;
        unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        if ((rc = MQTTClient_publishMessage(_client, _topic_name.c_str(), &_pubmsg, &_token)) != MQTTCLIENT_SUCCESS)
        {
            printf("Failed to publish message, return code %d\n", rc);
        }
        rc = MQTTClient_waitForCompletion(_client, _token, TIMEOUT);
        proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time;
        return proc_time;
    }

    ~TestPublisher(){
        int rc;
        if ((rc = MQTTClient_disconnect(_client, 10000)) != MQTTCLIENT_SUCCESS)
            printf("Failed to disconnect, return code %d\n", rc);
        MQTTClient_destroy(&_client);
    }

private:
    MQTTClient _client;
    MQTTClient_message _pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken _token;
};
