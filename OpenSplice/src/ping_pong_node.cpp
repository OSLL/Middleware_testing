#include "../../interface/ping_pong_interface.hpp"
#include "gen/TestData_DCPS.hpp"

class TestPingPongNode: public TestMiddlewarePingPong<TestDataType>{
public:
    TestPingPongNode(
            std::string &topic1, std::string &topic2, int msgCount, int prior,
            int cpu_index, std::string &filename, int topic_priority, int msInterval, int msgSize, bool isFirst
            ):
            TestMiddlewarePingPong<TestDataType>(topic1, topic2, msgCount, prior, cpu_index, filename, topic_priority,
                    msInterval, msgSize, isFirst),
            _dp(org::opensplice::domain::default_id()),
            _provider("file://QoS.xml", "TestProfile"),
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
            auto msg = samples.begin()->data();
            write_received_msg(msg);
            std::cout<< msg.id()<<std::endl;
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
        if (_isFirst){
            std::string data(size, 'b');
            unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            TestDataType msg(id, cur_time, std::vector<char>(data.begin(), data.end()));
            _dw.write(msg);}
        else{
            std::string data(size, 'a');
            unsigned long cur_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
            TestDataType msg(id, cur_time, std::vector<char>(data.begin(), data.end()));
            _dw.write(msg);}
    }

private:
    dds::domain::DomainParticipant _dp;
    dds::core::QosProvider _provider;
    dds::topic::Topic <TestDataType> _topic1;
    dds::topic::Topic <TestDataType> _topic2;
    dds::pub::Publisher _publisher;
    dds::pub::DataWriter <TestDataType> _dw;
    dds::sub::Subscriber _subscriber;
    dds::sub::DataReader <TestDataType> _dr;
};