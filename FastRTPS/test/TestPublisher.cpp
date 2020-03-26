#include "TestPublisher.h"
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/Domain.h>
#include <fastrtps/types/DynamicTypeBuilderFactory.h>
#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/types/DynamicTypeBuilder.h>
#include <fastrtps/types/DynamicTypeBuilderPtr.h>
#include <fastrtps/types/DynamicType.h>

#include <thread>

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;
using namespace eprosima::fastrtps::types;

TestPublisher::TestPublisher(std::string topic,  int msgCount, int prior, int cpu_index,
                  int min_msg_size, int max_msg_size, int step, int interval, int msgs_before_step)
    : TestMiddlewarePub(topic, msgCount, prior, cpu_index, min_msg_size, max_msg_size, step, interval, msgs_before_step)
    , mp_participant(nullptr)
    , mp_publisher(nullptr)
    , m_DynType(DynamicType_ptr(nullptr))
{
    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_struct_builder();
    builder->add_member(0, "id", DynamicTypeBuilderFactory::get_instance()->create_int16_type());
    builder->add_member(1, "sent_time", DynamicTypeBuilderFactory::get_instance()->create_uint64_type());
    builder->add_member(2, "data", DynamicTypeBuilderFactory::get_instance()->create_string_type(max_msg_size));

    DynamicType_ptr dynType(builder->build());
    m_DynType.SetDynamicType(dynType);
    m_DynMsg = DynamicDataFactory::get_instance()->create_data(dynType);

    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 0;
    std::string name = "FastRTPSTest_pub" + std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
    PParam.rtps.setName(name.c_str());
    mp_participant = Domain::createParticipant(PParam);

    //REGISTER THE TYPE
    Domain::registerDynamicType(mp_participant, &m_DynType);

    //CREATE THE PUBLISHER
//    PublisherAttributes Wparam;
    Wparam.topic.topicKind = NO_KEY;
    Wparam.topic.topicDataType = dynType->get_name();
    Wparam.topic.topicName = topic;

    Wparam.topic.historyQos.kind = KEEP_ALL_HISTORY_QOS;
    Wparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    Wparam.qos.m_publishMode.kind = ASYNCHRONOUS_PUBLISH_MODE;

    mp_publisher = Domain::createPublisher(mp_participant,Wparam,(PublisherListener*)&m_listener);
}

TestPublisher::~TestPublisher()
{
    Domain::removeParticipant(mp_participant);

    DynamicDataFactory::get_instance()->delete_data(m_DynMsg);

    Domain::stopAll();
}

void TestPublisher::PubListener::onPublicationMatched(
        Publisher* /*pub*/,
        MatchingInfo& info)
{
    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        firstConnected = true;
        std::cout << "Publisher matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Publisher unmatched" << std::endl;
    }
}

void TestPublisher::publish(short id, unsigned size) {
    unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    std::string data(size, 'a');
    m_DynMsg->set_int16_value(id, 0);
    m_DynMsg->set_uint64_value(cur_time, 1);
    m_DynMsg->set_string_value(data, 2);
    mp_publisher->write((void*)m_DynMsg);
}

