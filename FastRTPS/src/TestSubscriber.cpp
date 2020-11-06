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

TestSubscriber::TestSubscriber(std::string &topic, int msgCount, int prior, int cpu_index, std::string &filename, int topic_priority, int max_msg_size)
    : TestMiddlewareSub(topic, msgCount, prior, cpu_index, filename, topic_priority)
    , mp_participant(nullptr)
    , mp_subscriber(nullptr)
    , m_listener(this)
    , m_DynType(DynamicType_ptr(nullptr))
{
    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 0;
    std::string name = "FastRTPSTest_sub" + std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
    PParam.rtps.setName(name.c_str());
    mp_participant = Domain::createParticipant(PParam);

    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_struct_builder();
    builder->add_member(0, "id", DynamicTypeBuilderFactory::get_instance()->create_int16_type());
    builder->add_member(1, "sent_time", DynamicTypeBuilderFactory::get_instance()->create_uint64_type());
    builder->add_member(2, "data", DynamicTypeBuilderFactory::get_instance()->create_string_type(max_msg_size));

    dynType = builder->build();
    m_DynType.SetDynamicType(dynType);

    //REGISTER THE TYPE
    Domain::registerDynamicType(mp_participant, &m_DynType);

    //CREATE THE SUBSCRIBER
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = dynType->get_name();

    Rparam.topic.historyQos.kind = KEEP_ALL_HISTORY_QOS;
    Rparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;

    Rparam.topic.topicName = topic;
    for(int i=0; i<_msgs.size(); ++i)
        _msgs[i] = DynamicDataFactory::get_instance()->create_data(dynType);

    mp_subscriber = Domain::createSubscriber(mp_participant,Rparam,(SubscriberListener*)&m_listener);
}

TestSubscriber::~TestSubscriber()
{
    Domain::removeParticipant(mp_participant);

    for(int i=0; i<_msgs.size(); ++i)
        DynamicDataFactory::get_instance()->delete_data(_msgs[i]);

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
    unsigned proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    if (sub->takeNextData((void*)parent->_msgs[n_msgs], &m_info))
    {
        if (m_info.sampleKind == ALIVE)
        {
            proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - proc_time;
            parent->write_received_msg(parent->_msgs[n_msgs], proc_time);
            this->n_msgs++;
        }
    }
}

bool TestSubscriber::receive() {
    int rec_count = m_listener.n_msgs - m_listener.rec_before;
    m_listener.rec_before = m_listener.n_msgs;
    return rec_count;
}

short TestSubscriber::get_id(DynamicData* &msg) {
    if(msg == nullptr)
	return 0;
    short id;
    msg->get_int16_value(id, 0);
    return id;
}

unsigned long TestSubscriber::get_timestamp(DynamicData* &msg) {
    if(msg == nullptr)
	return 0;
    unsigned long timestamp;
    msg->get_uint64_value(timestamp, 1);
    return timestamp;
}
