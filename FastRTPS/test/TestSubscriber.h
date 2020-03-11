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

class TestSubscriber
{
    public:

        TestSubscriber();

        virtual ~TestSubscriber();

        bool init(std::string topic, int m_count, int max_msglen);

        void run();

        void run(
                uint32_t number);

        std::tuple<std::vector<std::string>, std::vector<unsigned long>> receive();
    private:

        int m_count;

        eprosima::fastrtps::Participant* mp_participant;

        eprosima::fastrtps::Subscriber* mp_subscriber;

    public:

        class SubListener : public eprosima::fastrtps::SubscriberListener
        {
            public:
                SubListener()
                    : n_matched(0)
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

		std::vector<std::string> rec_msgs;
		std::vector<unsigned long> rec_time;
        } m_listener;

    private:

        eprosima::fastrtps::types::DynamicPubSubType m_DynType;
};

#endif /* TESTSUBSCRIBER_H_ */
