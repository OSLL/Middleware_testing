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

TestSubscriber::TestSubscriber(std::string topic, int msgCount, int prior, int cpu_index, int max_msg_size, std::string res_filename)
    : TestMiddlewareSub(topic, msgCount, prior, cpu_index, max_msg_size, res_filename)
    , mp_participant(nullptr)
    , mp_subscriber(nullptr)
    , m_listener(SubListener(this))
    , m_DynType(DynamicType_ptr(nullptr))
    , rec_before(0)
{
    setQoS("qos.json");
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
    m_listener.m_DynMsg = DynamicDataFactory::get_instance()->create_data(dynType);

    //REGISTER THE TYPE
    Domain::registerDynamicType(mp_participant, &m_DynType);

    //CREATE THE SUBSCRIBER
//    SubscriberAttributes Rparam;
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = dynType->get_name();
    Rparam.topic.topicName = topic;

/*    Rparam.topic.historyQos.kind = KEEP_ALL_HISTORY_QOS;
    Rparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    Rparam.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
*/
    mp_subscriber = Domain::createSubscriber(mp_participant,Rparam,(SubscriberListener*)&m_listener);
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
            parent->rec_time[n_msgs] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            std::string message;
            short id;
            unsigned long sent_time;
            m_DynMsg->get_int16_value(id, 0);
            m_DynMsg->get_uint64_value(sent_time, 1);
            parent->msgs[n_msgs].first = id;
            parent->msgs[n_msgs].second = sent_time;
            this->n_msgs++;

            //std::cout << "Message: " << message << " RECEIVED" << std::endl;
        }
    }
}

int TestSubscriber::receive() {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    int rec_count = m_listener.n_msgs - rec_before;
    rec_before = m_listener.n_msgs;
    return rec_count;
}

void TestSubscriber::setQoS(std::string filename) {
    nlohmann::json qos;
    std::ifstream file(filename);
    if(!file.is_open()) {
        std::cout << "Cannot open qos file " << filename << std::endl;
        return;
    }
    file >> qos;
    file.close();
    if(qos["reliability"] != nullptr)
        if(qos["reliability"] == "RELIABLE")
            Rparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
        else Rparam.qos.m_reliability.kind = BEST_EFFORT_RELIABILITY_QOS;
    if(qos["history"] != nullptr)
        if(qos["history"] == "KEEP_ALL")
            Rparam.topic.historyQos.kind = KEEP_ALL_HISTORY_QOS;
        else Rparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
}
