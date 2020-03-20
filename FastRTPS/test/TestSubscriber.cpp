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

TestSubscriber::TestSubscriber()
    : mp_participant(nullptr)
    , mp_subscriber(nullptr)
    , m_DynType(DynamicType_ptr(nullptr))
{
}

bool TestSubscriber::init(std::string topic, int msgCount, int max_msglen)
{
    m_count = msgCount;
    m_listener.rec_msgs.resize(m_count);
    m_listener.rec_time.resize(m_count);

    ParticipantAttributes PParam;
    PParam.rtps.builtin.domainId = 0;
    PParam.rtps.setName("DynTest_sub");
    mp_participant = Domain::createParticipant(PParam);
    if (mp_participant == nullptr)
    {
        return false;
    }

    DynamicTypeBuilder_ptr created_builder = DynamicTypeBuilderFactory::get_instance()->create_string_builder(max_msglen);
    DynamicType_ptr dynType = DynamicTypeBuilderFactory::get_instance()->create_type(created_builder.get());
    m_DynType.SetDynamicType(dynType);
    m_listener.m_DynMsg = DynamicDataFactory::get_instance()->create_data(dynType);

    //REGISTER THE TYPE
    Domain::registerDynamicType(mp_participant, &m_DynType);

    //CREATE THE SUBSCRIBER
    SubscriberAttributes Rparam;
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = dynType->get_name();
    Rparam.topic.topicName = topic;

    Rparam.topic.historyQos.kind = KEEP_ALL_HISTORY_QOS;
    Rparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    Rparam.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;

    mp_subscriber = Domain::createSubscriber(mp_participant,Rparam,(SubscriberListener*)&m_listener);

    if (mp_subscriber == nullptr)
    {
        return false;
    }


    return true;
}

TestSubscriber::~TestSubscriber()
{
    Domain::removeParticipant(mp_participant);

    DynamicDataFactory::get_instance()->delete_data(m_listener.m_DynMsg);

    Domain::stopAll();
}

void TestSubscriber::SubListener::onSubscriptionMatched(
        Subscriber* /*sub*/,
        MatchingInfo& info)
{
    if (info.status == MATCHED_MATCHING)
    {
        n_matched++;
        std::cout << "Subscriber matched" << std::endl;
    }
    else
    {
        n_matched--;
        std::cout << "Subscriber unmatched" << std::endl;
    }
}

void TestSubscriber::SubListener::onNewDataMessage(
        Subscriber* sub)
{
    if (sub->takeNextData((void*)m_DynMsg, &m_info))
    {
        if (m_info.sampleKind == ALIVE)
        {
            rec_time[n_msgs] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            std::string message;
            m_DynMsg->get_string_value(message, MEMBER_ID_INVALID);
            rec_msgs[n_msgs] = message;
            this->n_msgs++;

            //std::cout << "Message: " << message << " RECEIVED" << std::endl;
        }
    }
}


void TestSubscriber::run()
{
    std::cout << "Subscriber running. Please press enter to stop the Subscriber" << std::endl;
    std::cin.ignore();
}

void TestSubscriber::run(
        uint32_t number)
{
    std::cout << "Subscriber running until " << number << "samples have been received" << std::endl;
    while (number > this->m_listener.n_msgs)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

std::tuple<std::vector<std::string>, std::vector<unsigned long>> TestSubscriber::receive() {
    int last_count = 0;
    while (m_count > this->m_listener.n_msgs)
    {
	last_count = m_listener.n_msgs;
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	if(m_listener.n_msgs == last_count && last_count != 0)
            break;
    }
    return std::make_tuple(m_listener.rec_msgs, m_listener.rec_time);
}
