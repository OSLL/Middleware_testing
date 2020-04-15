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

#include "../../interface/sub_interface.hpp"

class TestSubscriber : public TestMiddlewareSub<eprosima::fastrtps::types::DynamicData*>
{
    public:

        TestSubscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_prior, int max_msg_size);

        virtual ~TestSubscriber();

        bool receive();

	short get_id(eprosima::fastrtps::types::DynamicData* &msg);

	unsigned long get_timestamp(eprosima::fastrtps::types::DynamicData* &msg);

    private:

        eprosima::fastrtps::SubscriberAttributes Rparam;

        eprosima::fastrtps::Participant* mp_participant;

        eprosima::fastrtps::Subscriber* mp_subscriber;

    public:

        class SubListener : public eprosima::fastrtps::SubscriberListener
        {
            public:
                SubListener() {}
                SubListener(TestSubscriber* parent)
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

                eprosima::fastrtps::types::DynamicData* m_DynMsg;

                eprosima::fastrtps::SampleInfo_t m_info;


                uint32_t n_msgs;

                int rec_before;

                TestSubscriber* parent;
        };

        SubListener m_listener;

	eprosima::fastrtps::types::DynamicType_ptr dynType;
    private:

        eprosima::fastrtps::types::DynamicPubSubType m_DynType;
};

#endif /* TESTSUBSCRIBER_H_ */
