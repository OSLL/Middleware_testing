#include "../../interface/ping_pong_interface.hpp"
#include "gen/TestData_DCPS.hpp"

#define QOS_PATH "file:///home/andrew/work/middleware_project/dds_testing/OpenSplice/src/QoS.xml"

class TestPingPongNode: public TestMiddlewarePingPong<TestDataType>{
public:
    TestPingPongNode(
            std::string &topic1, std::string &topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst
            ):
            TestMiddlewarePingPong<TestDataType>(topic1, topic2, msgCount, prior, cpu_index, filename, topic_priority,
                    msInterval, msgSize, isFirst),
            _dp(org::opensplice::domain::default_id()),
            _provider(QOS_PATH, "TestProfile"),
            _topic1(_dp, _topic_name1, _provider.topic_qos()),
            _topic2(_dp, _topic_name2, _provider.topic_qos()),
            _publisher(_dp),
            _dw(_publisher, _topic1, _provider.datawriter_qos()),
            _subscriber(_dp),
            _dr(_subscriber, _topic2, _provider.datareader_qos())
            {
                if (!_isFirst){
                    _dw = dds::pub::DataWriter<TestDataType>(_publisher, _topic2, _provider.datawriter_qos());
                    _dr = dds::sub::DataReader<TestDataType>(_subscriber, _topic1, _provider.datareader_qos());
                }
            }

    TestPingPongNode(std::string &topic1, std::string topic2, int msgCount, int prior,
                     int cpu_index, std::string &filename, int topic_priority, int msInterval,
                     int msgSizeMin, int msgSizeMax, int step, int before_step, bool isFirst):
            TestMiddlewarePingPong<TestDataType>(topic1, topic2, msgCount, prior, cpu_index, filename, topic_priority,
                                                 msInterval, msgSizeMin, msgSizeMax, step, before_step, isFirst),
            _dp(org::opensplice::domain::default_id()),
            _provider(QOS_PATH, "TestProfile"),
            _topic1(_dp, _topic_name1, _provider.topic_qos()),
            _topic2(_dp, _topic_name2, _provider.topic_qos()),
            _publisher(_dp),
            _dw(_publisher, _topic1, _provider.datawriter_qos()),
            _subscriber(_dp),
            _dr(_subscriber, _topic2, _provider.datareader_qos())
                    {
                        if (!_isFirst){
                            _dw = dds::pub::DataWriter<TestDataType>(_publisher, _topic2, _provider.datawriter_qos());
                            _dr = dds::sub::DataReader<TestDataType>(_subscriber, _topic1, _provider.datareader_qos());
                        }
                    }

    bool receive() override {
        auto samples = _dr.select().max_samples(1).state(dds::sub::status::DataState::new_data()).take();
        if(samples.length() > 0){
            unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>
                    (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            auto msg = samples.begin()->data();
            write_received_msg(msg);
            _read_msg_time[get_id(msg)]=std::chrono::duration_cast<std::chrono::nanoseconds>
                    (std::chrono::high_resolution_clock::now().time_since_epoch()).count()-cur_time;
            // std::cout<< msg.id()<<std::endl;
            return true;
        }
        return false;
    }

    short get_id(TestDataType &msg) override {
        return msg.id();
    }

    unsigned long get_timestamp(TestDataType &msg) override {
        return msg.timestamp();
    }

    void publish(short id, unsigned size) override {
        //std::cout << id;
        if (_isFirst){
            std::string data(size, 'b');
            write_to_topic(id, data);}
        else{
            std::string data(size, 'a');
            write_to_topic(id, data);}
    }

private:

    void write_to_topic(short id, std::string &data){
        unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>
                (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        TestDataType msg(id, cur_time, std::vector<char>(data.begin(), data.end()));
        cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>
                (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        _dw.write(msg);
        auto proc_time = std::chrono::duration_cast<std::chrono::nanoseconds>
                (std::chrono::high_resolution_clock::now().time_since_epoch()).count() - cur_time;
        _write_msg_time[id]=proc_time;
    }

    dds::domain::DomainParticipant _dp;
    dds::core::QosProvider _provider;
    dds::topic::Topic <TestDataType> _topic1;
    dds::topic::Topic <TestDataType> _topic2;
    dds::pub::Publisher _publisher;
    dds::pub::DataWriter <TestDataType> _dw;
    dds::sub::Subscriber _subscriber;
    dds::sub::DataReader <TestDataType> _dr;
};
