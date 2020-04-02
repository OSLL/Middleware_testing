#include "TestSubscriber.h"
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/Domain.h>
#include <fastrtps/types/DynamicTypeBuilderFactory.h>
#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/types/DynamicTypeBuilder.h>
#include <fastrtps/types/DynamicTypeBuilderPtr.h>
#include <fastrtps/types/DynamicType.h>

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;
using namespace eprosima::fastrtps::types;

TestSubscriber::TestSubscriber(std::vector<std::string> &topics, int msgCount, int prior, int cpu_index, std::vector<std::string> &res_filenames, int max_msg_size)
    : TestMiddlewareSub(topics, msgCount, prior, cpu_index, res_filenames)
    , mp_participant(nullptr)
    , mp_subscribers(topics.size())
    , m_listeners(topics.size())
    , m_DynType(DynamicType_ptr(nullptr))
{
    for(int i = 0; i < topics.size(); ++i)
        m_listeners[i] = SubListener(this, i);
    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 0;
    std::string name = "FastRTPSTest_sub" + std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
    PParam.rtps.setName(name.c_str());
    mp_participant = Domain::createParticipant(PParam);

    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_struct_builder();
    builder->add_member(0, "id", DynamicTypeBuilderFactory::get_instance()->create_int16_type());
    builder->add_member(1, "sent_time", DynamicTypeBuilderFactory::get_instance()->create_uint64_type());
    builder->add_member(2, "data", DynamicTypeBuilderFactory::get_instance()->create_string_type(max_msg_size));

    DynamicType_ptr dynType(builder->build());
    m_DynType.SetDynamicType(dynType);

    //REGISTER THE TYPE
    Domain::registerDynamicType(mp_participant, &m_DynType);

    //CREATE THE SUBSCRIBER
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = dynType->get_name();

    Rparam.topic.historyQos.kind = KEEP_ALL_HISTORY_QOS;
    Rparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;

    for(int i = 0; i < topics.size(); ++i) {
        Rparam.topic.topicName = topics[i];
        m_listeners[i].m_DynMsg = DynamicDataFactory::get_instance()->create_data(dynType);

        mp_subscribers[i] = Domain::createSubscriber(mp_participant,Rparam,(SubscriberListener*)&m_listeners[i]);
    }
}

TestSubscriber::~TestSubscriber()
{
    Domain::removeParticipant(mp_participant);

    for(int i = 0; i < m_listeners.size(); ++i)
        DynamicDataFactory::get_instance()->delete_data(m_listeners[i].m_DynMsg);

    Domain::stopAll();
}

void TestSubscriber::SubListener::onSubscriptionMatched(
        Subscriber* /*sub*/,
        MatchingInfo& info)
{
}

void TestSubscriber::SubListener::onNewDataMessage(
        Subscriber* sub)
{
    if (sub->takeNextData((void*)m_DynMsg, &m_info))
    {
        if (m_info.sampleKind == ALIVE)
        {
            parent->rec_time[subscr_n][n_msgs] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            short id;
            unsigned long sent_time;
            m_DynMsg->get_int16_value(id, 0);
            m_DynMsg->get_uint64_value(sent_time, 1);
            parent->msgs[subscr_n][n_msgs].first = id;
            parent->msgs[subscr_n][n_msgs].second = sent_time;
            this->n_msgs++;

        }
    }
}

int TestSubscriber::receive(int topic_id) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    int rec_count = m_listeners[topic_id].n_msgs - m_listeners[topic_id].rec_before;
    m_listeners[topic_id].rec_before = m_listeners[topic_id].n_msgs;
    return rec_count;
}

