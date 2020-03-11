#ifndef TESTPUBLISHER_H_
#define TESTPUBLISHER_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/participant/ParticipantListener.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/types/DynamicData.h>
#include <fastrtps/types/DynamicPubSubType.h>


class TestPublisher
{
    public:

        TestPublisher();

        virtual ~TestPublisher();

        bool init(std::string topic, int max_msglen);

	void publish(std::string &msg);

        void run(
                uint32_t number,
                uint32_t sleep);

    private:

        eprosima::fastrtps::Participant* mp_participant;

        eprosima::fastrtps::Publisher* mp_publisher;

        bool stop;

	int m_interval;

        class PubListener : public eprosima::fastrtps::PublisherListener
        {
            public:
                PubListener()
                    : n_matched(0)
                    , firstConnected(false)
                {}

                ~PubListener() override {}

                void onPublicationMatched(
                        eprosima::fastrtps::Publisher* pub,
                        eprosima::fastrtps::rtps::MatchingInfo& info) override;

                int n_matched;

                bool firstConnected;

        } m_listener;

        eprosima::fastrtps::types::DynamicData* m_DynMsg;

        eprosima::fastrtps::types::DynamicPubSubType m_DynType;
};



#endif /* TESTPUBLISHER_H_ */
