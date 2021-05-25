#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/Domain.h>
#include <fastrtps/types/DynamicTypeBuilderFactory.h>
#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/types/DynamicTypeBuilder.h>
#include <fastrtps/types/DynamicTypeBuilderPtr.h>
#include <fastrtps/types/DynamicType.h>
#include "TestPingPongNode.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;
using namespace eprosima::fastrtps::types;

TestPingPongNode::TestPingPongNode(
            std::string &topic1, std::string topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst) :
	    TestMiddlewarePingPong(topic1, topic2, msgCount, prior, cpu_index,
		        filename, topic_priority, msInterval, msgSize, isFirst)
            , mp_pparticipant(nullptr)
            , mp_sparticipant(nullptr)
            , mp_publisher(nullptr)
            , m_DynType(DynamicType_ptr(nullptr))
	    , m_slistener(this)
{
    init();
}

TestPingPongNode::TestPingPongNode(std::string &topic1, std::string topic2, int msgCount, int prior, int cpu_index,
                                   std::string &filename, int topic_priority, int msInterval, int msgSizeMin,
                                   int msgSizeMax, int step, int before_step, bool isFirst) :
            TestMiddlewarePingPong(topic1, topic2, msgCount, prior, cpu_index, filename, topic_priority,
                                   msInterval, msgSizeMin, msgSizeMax, step, before_step, isFirst)
            , mp_pparticipant(nullptr)
            , mp_sparticipant(nullptr)
            , mp_publisher(nullptr)
            , m_DynType(DynamicType_ptr(nullptr))
            , m_slistener(this)
{
    init();
}

void TestPingPongNode::init() {
    if(!_isFirst)
        std::swap(_topic_name1, _topic_name2);
    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_struct_builder();
    builder->add_member(0, "id", DynamicTypeBuilderFactory::get_instance()->create_int16_type());
    builder->add_member(1, "sent_time", DynamicTypeBuilderFactory::get_instance()->create_uint64_type());
    builder->add_member(2, "data", DynamicTypeBuilderFactory::get_instance()->create_string_type(_msgSize+2));

    DynamicType_ptr dynType(builder->build());
    m_DynType.SetDynamicType(dynType);
    m_DynMsg = DynamicDataFactory::get_instance()->create_data(dynType);

    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 0;
    std::string name = "FastRTPSTest_ping_pong" + std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
    PParam.rtps.setName(name.c_str());
    mp_pparticipant = Domain::createParticipant(PParam);
    name = "FastRTPSTest_ping_pong" + std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
    PParam.rtps.setName(name.c_str());
    mp_sparticipant = Domain::createParticipant(PParam);

    //REGISTER THE TYPE
    Domain::registerDynamicType(mp_pparticipant, &m_DynType);
    Domain::registerDynamicType(mp_sparticipant, &m_DynType);

    //CREATE THE PUBLISHER
    PublisherAttributes Wparam;
    Wparam.topic.topicKind = NO_KEY;
    Wparam.topic.topicDataType = dynType->get_name();
    Wparam.topic.topicName = _topic_name1;

    Wparam.topic.historyQos.kind = KEEP_ALL_HISTORY_QOS;
    Wparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    Wparam.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;

    mp_publisher = Domain::createPublisher(mp_pparticipant,Wparam,(PublisherListener*)&m_plistener);

    SubscriberAttributes Rparam;
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = dynType->get_name();

    Rparam.topic.historyQos.kind = KEEP_ALL_HISTORY_QOS;
    Rparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;

    Rparam.topic.topicName = _topic_name2;
    for(int i=0; i<_msgs.size(); ++i)
        _msgs[i] = DynamicDataFactory::get_instance()->create_data(dynType);

    mp_subscriber = Domain::createSubscriber(mp_sparticipant,Rparam,(SubscriberListener*)&m_slistener);
}

bool TestPingPongNode::receive(){
    int rec_count = m_slistener.n_msgs - m_slistener.rec_before;
    if(rec_count > 0)
        m_slistener.rec_before++;
    return rec_count;
}

short TestPingPongNode::get_id(DynamicData* &msg) {
    if(msg == nullptr)
        return 0;
    short id;
    msg->get_int16_value(id, 0);
    return id;
}

unsigned long TestPingPongNode::get_timestamp(DynamicData* &msg) {
    if(msg == nullptr)
        return 0;
    unsigned long timestamp;
    msg->get_uint64_value(timestamp, 1);
    return timestamp;
}

void TestPingPongNode::publish(short id, unsigned size) {
    std::string data(size, 'a');
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    m_DynMsg->set_int16_value(id, 0);
    m_DynMsg->set_uint64_value(cur_time, 1);
    m_DynMsg->set_string_value(std::string(size, 'a'), 2);
    cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>
            (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    mp_publisher->write((void*)m_DynMsg);
    _write_msg_time[id]=std::chrono::duration_cast<std::chrono::nanoseconds>
            (std::chrono::high_resolution_clock::now().time_since_epoch()).count()-cur_time;
}

void TestPingPongNode::PubListener::onPublicationMatched(
        Publisher* /*pub*/,
        MatchingInfo& info)
{
    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Publisher matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Publisher unmatched" << std::endl;
    }
}

void TestPingPongNode::SubListener::onSubscriptionMatched(
        Subscriber* /*sub*/,
        MatchingInfo& info)
{
}

void TestPingPongNode::SubListener::onNewDataMessage(
        Subscriber* sub)
{
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>
            (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    if (sub->takeNextData((void*)parent->_msgs[n_msgs], &m_info))
    {
        if (m_info.sampleKind == ALIVE)
        {
            auto proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>
                    (std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time;
            parent->_read_msg_time[n_msgs] = proc_time;
            parent->write_received_msg(parent->_msgs[n_msgs]);
            this->n_msgs++;
        }
    }
}

TestPingPongNode::~TestPingPongNode()
{
    Domain::removeParticipant(mp_pparticipant);
    Domain::removeParticipant(mp_sparticipant);

    for(int i=0; i<_msgs.size(); ++i)
        DynamicDataFactory::get_instance()->delete_data(_msgs[i]);

    DynamicDataFactory::get_instance()->delete_data(m_DynMsg);

    Domain::stopAll();
}

