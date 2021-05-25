#pragma once
#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/participant/ParticipantListener.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/types/DynamicData.h>
#include <fastrtps/types/DynamicPubSubType.h>
#include "../../interface/ping_pong_interface.hpp"
#include "TestPublisher.h"
#include "TestSubscriber.h"


class TestPingPongNode : public TestMiddlewarePingPong<eprosima::fastrtps::types::DynamicData*> {
public:
    TestPingPongNode(
            std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst);

    TestPingPongNode(
            std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSizeMin, int msgSizeMax,
            int step, int before_step, bool isFirst);

    virtual bool receive();
    virtual short get_id(eprosima::fastrtps::types::DynamicData* &msg);
    virtual unsigned long get_timestamp(eprosima::fastrtps::types::DynamicData* &msg);
    virtual void publish(short id, unsigned size);

    ~TestPingPongNode();

private:

    void init();

    eprosima::fastrtps::Participant* mp_pparticipant;
    
    eprosima::fastrtps::Participant* mp_sparticipant;

    eprosima::fastrtps::Publisher* mp_publisher;

    eprosima::fastrtps::Subscriber* mp_subscriber;

    class PubListener : public eprosima::fastrtps::PublisherListener
    {
        public:
            PubListener()
                : n_matched(0)
            {}

	    ~PubListener() override {}

            void onPublicationMatched(
                    eprosima::fastrtps::Publisher* pub,
                    eprosima::fastrtps::rtps::MatchingInfo& info) override;

            int n_matched;

    } m_plistener;

    class SubListener : public eprosima::fastrtps::SubscriberListener
    {
        public:
            SubListener() {}
            SubListener(TestPingPongNode* parent)
                : parent(parent)
                , n_msgs(0)
                , rec_before(0)
            {}

            ~SubListener() override {}

            void onSubscriptionMatched(
                    eprosima::fastrtps::Subscriber* sub,
                    eprosima::fastrtps::rtps::MatchingInfo& info) override;

            void onNewDataMessage(
                    eprosima::fastrtps::Subscriber* sub) override;

            eprosima::fastrtps::SampleInfo_t m_info;


            uint32_t n_msgs;
            int rec_before;

            TestPingPongNode* parent;
    } m_slistener;

    eprosima::fastrtps::types::DynamicData* m_DynMsg;

    eprosima::fastrtps::types::DynamicPubSubType m_DynType;

};
