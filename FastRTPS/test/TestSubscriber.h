#ifndef TESTSUBSCRIBER_H_
#define TESTSUBSCRIBER_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastrtps/participant/ParticipantListener.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/types/DynamicData.h>
#include <fastrtps/types/DynamicPubSubType.h>

#include "test_interface.hpp"

class TestSubscriber : public TestMiddlewareSub
{
    public:

        TestSubscriber(std::string topic, int msgCount=0, int prior = -1, int cpu_index = -1, int max_msg_size=64000, std::string res_filename="res.json");

        virtual ~TestSubscriber();

        int receive();

        void setQoS(std::string filename);

    private:

        eprosima::fastrtps::SubscriberAttributes Rparam;

        int rec_before;

        eprosima::fastrtps::Participant* mp_participant;

        eprosima::fastrtps::Subscriber* mp_subscriber;

    public:

        class SubListener : public eprosima::fastrtps::SubscriberListener
        {
            public:
                SubListener(TestSubscriber* parent)
                    : parent(parent)
                    , n_matched(0)
                    , n_msgs(0)
                {}

                ~SubListener() override {}

                void onSubscriptionMatched(
                        eprosima::fastrtps::Subscriber* sub,
                        eprosima::fastrtps::rtps::MatchingInfo& info) override;

                void onNewDataMessage(
                        eprosima::fastrtps::Subscriber* sub) override;

                eprosima::fastrtps::types::DynamicData* m_DynMsg;

                eprosima::fastrtps::SampleInfo_t m_info;

                int n_matched;

                uint32_t n_msgs;

                TestSubscriber* parent;
        } m_listener;

    private:

        eprosima::fastrtps::types::DynamicPubSubType m_DynType;
};

#endif /* TESTSUBSCRIBER_H_ */
