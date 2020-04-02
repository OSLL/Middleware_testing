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

class TestSubscriber : public TestMiddlewareSub
{
    public:

        TestSubscriber(std::vector<std::string> &topics, int msgCount, int prior, int cpu_index, std::vector<std::string> &res_filenames, int max_msg_size);

        virtual ~TestSubscriber();

        int receive(int topic_id);

    private:

        eprosima::fastrtps::SubscriberAttributes Rparam;

        eprosima::fastrtps::Participant* mp_participant;

        std::vector<eprosima::fastrtps::Subscriber*> mp_subscribers;

    public:

        class SubListener : public eprosima::fastrtps::SubscriberListener
        {
            public:
                SubListener() {}
                SubListener(TestSubscriber* parent, int n)
                    : parent(parent)
                    , n_msgs(0)
                    , rec_before(0)
                    , subscr_n(n)
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

                int subscr_n;

                TestSubscriber* parent;
        };

        std::vector<SubListener> m_listeners;

    private:

        eprosima::fastrtps::types::DynamicPubSubType m_DynType;
};

#endif /* TESTSUBSCRIBER_H_ */
