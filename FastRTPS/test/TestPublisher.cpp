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

TestPublisher::TestPublisher()
    : mp_participant(nullptr)
    , mp_publisher(nullptr)
    , m_DynType(DynamicType_ptr(nullptr))
{
}

bool TestPublisher::init(std::string topic, int max_msglen)
{
    DynamicTypeBuilder_ptr created_builder = DynamicTypeBuilderFactory::get_instance()->create_string_builder(max_msglen);
    DynamicType_ptr dynType = DynamicTypeBuilderFactory::get_instance()->create_type(created_builder.get());
    m_DynType.SetDynamicType(dynType);
    m_DynMsg = DynamicDataFactory::get_instance()->create_data(dynType);

    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 0;
    PParam.rtps.setName("DynTest_pub");
    mp_participant = Domain::createParticipant(PParam);

    if (mp_participant == nullptr)
    {
        return false;
    }

    //REGISTER THE TYPE
    Domain::registerDynamicType(mp_participant, &m_DynType);

    //CREATE THE PUBLISHER
    PublisherAttributes Wparam;
    Wparam.topic.topicKind = NO_KEY;
    Wparam.topic.topicDataType = dynType->get_name();
    Wparam.topic.topicName = topic;

    Wparam.topic.historyQos.kind = KEEP_ALL_HISTORY_QOS;
    Wparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    Wparam.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;

    mp_publisher = Domain::createPublisher(mp_participant,Wparam,(PublisherListener*)&m_listener);
    if (mp_publisher == nullptr)
    {
        return false;
    }

    return true;

}

TestPublisher::~TestPublisher()
{
    Domain::removeParticipant(mp_participant);

    DynamicDataFactory::get_instance()->delete_data(m_DynMsg);

    Domain::stopAll();

    std::cout << "oooops.." << std::endl;
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

void TestPublisher::run(
        uint32_t samples,
        uint32_t sleep)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    int i = 0;
    std::string msg(10000, 'f');
    while(i < samples) {
        publish(msg);
        ++i;
    }
}

void TestPublisher::publish(std::string &msg) {
    m_DynMsg->set_string_value(msg);
    mp_publisher->write((void*)m_DynMsg);
}
