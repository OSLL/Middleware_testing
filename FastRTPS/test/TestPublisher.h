#ifndef TESTPUBLISHER_H_
#define TESTPUBLISHER_H_

#include <fastrtps/fastrtps_fwd.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/participant/ParticipantListener.h>
#include <fastrtps/participant/Participant.h>
#include <fastrtps/types/DynamicData.h>
#include <fastrtps/types/DynamicPubSubType.h>

#include "../../interface/pub_interface.hpp"

class TestPublisher : public TestMiddlewarePub
{
    public:

        TestPublisher(std::string topic,  int msgCount, int prior, int cpu_index,
                  int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step,
		  std::string &filename, int topic_priority);

        virtual ~TestPublisher();

	unsigned long publish(short id, unsigned size);

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
