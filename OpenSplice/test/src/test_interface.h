#include <string>
#include <chrono>
#include <ctime>

template <class Publisher>
class TestMiddlewarePub
{

    /*template <class QoS>
    struct IQoS {
        QoS qos;
    };*/
 
public:
 
    explicit ITestDDS(int msInterval, int byteSize, int msgCount, std::string topic) :
    _msInterval(msInterval),
    _byteSize(byteSize),
    _msgCount(msgCount),
    _topic(topic) {
        _publisher = createPublisher(topic);
    };
 
    virtual Publisher createPublisher(std::string topic)=0;

    virtual void publish(std::string msg)=0;
 
//    virtual IQoS<class T> createQoS()=0;
 
//     метод для реализации тестов
    void test0(){
        int counter;
        std::this_thread::sleep_for(std::chrono::seconds(4));
        while(counter < _msgCount) {
            publish('message');
            std::this_thread::sleep_for(std::chrono::mseconds(_msInterval));
        }
    };
 
private:
    int _msInterval = 0;
    int _byteSize = 0;
    int _msgCount = 0;
    std::string _topic;
    Publisher _publisher;
};

template <class Subscriber>
class TestMiddlewareSub
{

    /*template <class QoS>
    struct IQoS {
        QoS qos;
    };*/

public:

    explicit ITestDDS(int msInterval, int byteSize, int msgCount, std::string topic) :
            _msInterval(msInterval),
            _byteSize(byteSize),
            _msgCount(msgCount),
            _topic(topic) {
        _subscriber = createSubscriber(topic);
    };

    virtual Subscriber createSubscriber(std::string topic)=0;

    virtual unsigned int receive(Publisher publisher)=0;  //возвращает количетство принятых сообщений

//    virtual IQoS<class T> createQoS()=0;

//     метод для реализации тестов
    void test0(){
        int counter;
        while(counter < _msgCount) {
                counter += receive();
        }
    };

private:
    int _msInterval = 0;
    int _byteSize = 0;
    int _msgCount = 0;
    std::string _topic;
    Subscriber _subscriber;
};