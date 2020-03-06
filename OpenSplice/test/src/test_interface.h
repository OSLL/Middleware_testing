#include <string>
#include <chrono>
#include <ctime>
#include <vector>

template <class Publisher>
class TestMiddlewarePub
{
public:
 
    explicit ITestDDS(int msInterval, int byteSize, int msgCount, std::string topic) :
    _msInterval(msInterval),
    _byteSize(byteSize),
    _msgCount(msgCount),
    _topic(topic) {
        _publisher = createPublisher(topic);
    };

    void toJson(std::vector<?> time){
        //перенести id сообщение и его время получения/отправки в json
    }
 
    virtual Publisher createPublisher(std::string topic)=0;

    virtual void publish(std::string msg)=0;

    virtual void setQoS(std::string filename)=0;	//считывать наверное тоже из json, так как будет разные конфигурации QoS
 
 
private:
    int _msInterval = 0;
    int _byteSizeMin = 0;
    int _byteSizeMax = 0;
    int _step = 0;
    int _msg_count_befor_step = 0;
    int _msgCount = 0;
    int _priority = -1 //not stated
    int _cpu_index = -1; //not stated
    std::string _topic;
    Publisher _publisher;
};

template <class Subscriber>
class TestMiddlewareSub
{
public:

    explicit ITestDDS(int msInterval, int byteSize, int msgCount, std::string topic) :
            _msInterval(msInterval),
            _byteSize(byteSize),
            _msgCount(msgCount),
            _topic(topic) {
        _subscriber = createSubscriber(topic);
    };

    virtual Subscriber createSubscriber(std::string topic)=0;

    virtual std::vector<std::string> receive(Publisher publisher)=0;  //возвращает вектор принятых сообщений

    virtual void setQoS(std::string filename)=0;


private:
    int _msgCount = 0;
    int _priority = -1 //not stated
    int _cpu_index = -1; //not stated
    std::string _topic;
    Subscriber _subscriber;
};

QoS, 
