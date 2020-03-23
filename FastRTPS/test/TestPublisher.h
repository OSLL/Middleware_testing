#ifndef TESTPUBLISHER_H_
#define TESTPUBLISHER_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/participant/ParticipantListener.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/types/DynamicData.h>
#include <fastrtps/types/DynamicPubSubType.h>

#include "test_interface.hpp"

class TestPublisher : public TestMiddlewarePub
{
    public:

        TestPublisher(std::string topic,  int msgCount=0, int prior = -1, int cpu_index = -1,
                  int min_msg_size=50, int max_msg_size=64000, int step=0, int interval = 0, int msgs_before_step = 100);

        virtual ~TestPublisher();

	void publish(short id, unsigned size);

        void setQoS(std::string filename);

    private:

        eprosima::fastrtps::Participant* mp_participant;

        eprosima::fastrtps::Publisher* mp_publisher;

	int m_interval;

        eprosima::fastrtps::PublisherAttributes Wparam;

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
