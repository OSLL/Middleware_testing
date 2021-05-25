#include <qpid/messaging/Connection.h>
#include <qpid/messaging/Message.h>
#include <qpid/messaging/Sender.h>
#include <qpid/messaging/Receiver.h>
#include <qpid/messaging/Session.h>

#include"../../interface/pub_interface.hpp"
#include"../../interface/sub_interface.hpp"
#include"../../interface/ping_pong_interface.hpp"
#include<argparse/argparse.hpp>

#include<string>
#include<cstring>
#include<chrono>
#include<ctime>
#include<iostream>

#define PARAM_STRING "; {create: always, node: {type: topic, durable: True},\
link: {durable: True, reliability: at-least-once}}"

struct Message{
	unsigned long int timestamp;
	short id;
};

class Publisher: public TestMiddlewarePub{
private:
    qpid::messaging::Connection connection;
    qpid::messaging::Session session;
    qpid::messaging::Sender pub;
public:
    Publisher(std::string& address,
        std::string& topic,
        int msgCount, 
        int prior, 
        int cpu_index,
        int min_msg_size, 
        int max_msg_size, 
        int step, 
        int interval, 
        int msgs_before_step,
        std::string &filename, 
        int topic_priority
        ): TestMiddlewarePub(topic, msgCount, prior, cpu_index, min_msg_size,
                            max_msg_size, step, interval, msgs_before_step, 
                            filename, topic_priority)
    {
        std::string param = topic + PARAM_STRING;
        qpid::messaging::Connection connection(address, "");

        connection.open();
        session = connection.createSession();
        pub = session.createSender(param);

    }

    ~Publisher(){
        connection.close();
    }

    unsigned long publish(short id, unsigned size) override{
        std::string str(size,'a');
        unsigned long time=std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count();
        
        qpid::messaging::Message message;
        qpid::types::Variant::Map content;
        content["id"] = id;
        content["time"] = time;
        content["data"] = str;
        message.setContentObject(content);
        pub.send(message, true);

        time=std::chrono::duration_cast<std::chrono::
            nanoseconds>(std::chrono::high_resolution_clock::
            now().time_since_epoch()).count()-time;
        return time;
    }
};


template<class MsgType>
class Subscriber: public TestMiddlewareSub<MsgType>{
private:
    qpid::messaging::Connection connection;
    qpid::messaging::Session session;
    qpid::messaging::Receiver sub;
public:
    Subscriber(std::string& address,
        std::string &topic, 
        int msgCount, 
        int prior,
        int cpu_index, 
        std::string &filename, 
        int topic_priority
        ): TestMiddlewareSub<MsgType>(topic, msgCount, prior,
                            cpu_index, filename, topic_priority)
    {
        std::string param = topic + PARAM_STRING;
        qpid::messaging::Connection connection(address, "");

        connection.open();
        session = connection.createSession();
        sub = session.createReceiver(param);
    }

    ~Subscriber(){
        connection.close();
    }
        
    short get_id(MsgType &msg) override{
        return msg.id;
    }
        
    unsigned long get_timestamp(MsgType &msg) override{
        return msg.timestamp;
    }
	
    bool receive() override{
        unsigned long time=std::chrono::duration_cast<std::chrono::
                    nanoseconds>(std::chrono::high_resolution_clock::
                    now().time_since_epoch()).count();
        qpid::messaging::Message message;
        bool get=sub.fetch(message, qpid::messaging::Duration::IMMEDIATE);
        if(get){
            qpid::types::Variant::Map content = message.getContentObject().asMap();
            Message msg = {content["time"], content["id"]};
            std::string str(((std::string)content["data"]).c_str());

            session.acknowledge();

            time=std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count()-time;
	    TestMiddlewareSub<MsgType>::write_received_msg(msg,time);
        }
        return get;
    }
};


template<class MsgType>
class PingPong: public TestMiddlewarePingPong<MsgType>{
private:
    qpid::messaging::Connection connection;
    qpid::messaging::Session session;
    qpid::messaging::Receiver sub;
    qpid::messaging::Sender pub;
public:
    PingPong(std::string& address,
        std::string& topic1,
        std::string& topic2,
        int msgCount,
        int prior,
        int cpu_index,
        std::string &filename, 
        int topic_priority,
        int interval, 
        int msg_size, 
        bool isFirst
        ): TestMiddlewarePingPong<MsgType>(topic1, topic2, msgCount, prior, cpu_index, filename,
                                topic_priority, interval, msg_size, isFirst)
    {
        std::string param1 = topic1 + PARAM_STRING;
        std::string param2 = topic2 + PARAM_STRING;
        qpid::messaging::Connection connection(address, "");

        connection.open();
        session = connection.createSession();
        if(isFirst){
            pub = session.createSender(param1);
            sub = session.createReceiver(param2);
        }else{
            pub = session.createSender(param2);
            sub = session.createReceiver(param1);
        }
    }
    
    PingPong(std::string& address,
        std::string& topic1,
        std::string& topic2,
        int msgCount,
        int prior,
        int cpu_index,
        std::string &filename, 
        int topic_priority,
        int interval, 
        int msgSizeMin,
        int msgSizeMax,
        int step,
        int before_step, 
        bool isFirst
        ): TestMiddlewarePingPong<MsgType>(topic1, topic2, msgCount, prior, cpu_index, filename,
                                topic_priority, interval, msgSizeMin, msgSizeMax, step, before_step, isFirst)
    {
        std::string param1 = topic1 + PARAM_STRING;
        std::string param2 = topic2 + PARAM_STRING;
        qpid::messaging::Connection connection(address, "");

        connection.open();
        session = connection.createSession();
        if(isFirst){
            pub = session.createSender(param1);
            sub = session.createReceiver(param2);
        }else{
            pub = session.createSender(param2);
            sub = session.createReceiver(param1);
        }
    }

    ~PingPong(){
        connection.close();
    }

    void publish(short id, unsigned size) override{
        std::string str(size,'a');
        
        qpid::messaging::Message message;
        qpid::types::Variant::Map content;
        unsigned long time=std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count();
        content["id"] = id;
        content["time"] = std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count();
        content["data"] = str;
        message.setContentObject(content);
        time=std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count() - time;
        pub.send(message, true);
        TestMiddlewarePingPong<MsgType>::_write_msg_time[id] = time;
    }

        
    short get_id(MsgType &msg) override{
        return msg.id;
    }
        
    unsigned long get_timestamp(MsgType &msg) override{
        return msg.timestamp;
    }
        
    bool receive() override{
        qpid::messaging::Message message;
        bool get=sub.fetch(message, qpid::messaging::Duration::IMMEDIATE);
	if(get){ 
            unsigned long time=std::chrono::duration_cast<std::chrono::
                    nanoseconds>(std::chrono::high_resolution_clock::
                    now().time_since_epoch()).count();
            qpid::types::Variant::Map content = message.getContentObject().asMap();
            Message msg = {content["time"], content["id"]};
            std::string str(((std::string)content["data"]).c_str());
            session.acknowledge();
            time=std::chrono::duration_cast<std::chrono::
                    nanoseconds>(std::chrono::high_resolution_clock::
                    now().time_since_epoch()).count() - time;
            TestMiddlewarePingPong<MsgType>::write_received_msg(msg);
            TestMiddlewarePingPong<MsgType>::_read_msg_time[msg.id] = time;
        }
        return get;
    }
};

int main(int argc, char** argv){
    argparse::ArgumentParser program("PubSub");

    program.add_argument("-c","--config")
                    .required()
                    .help("is a config for testing");
    program.add_argument("-t","--type")
                    .required()
                    .help("is a type of the node: publisher, subscriber or ping_pong");
    program.add_argument("--first")
	            .implicit_value(true)
                    .default_value(false)
                    .help("is a flag for first in ping_pong test");
    program.add_argument("-a","--address")
                    .required()
                    .help(" server address");

    try{
        program.parse_args(argc, argv);
    }catch(const std::runtime_error& err){
        std::cout << err.what() << std::endl;
        std::cout << program<<std::endl;
        return 1;
    }
    auto type = program.get<std::string>("-t");
    auto conf_path = program.get<std::string>("-c");
    auto address = program.get<std::string>("-a");

    std::ifstream file(conf_path);
    if(!file){
        std::cout<<"Can't open file "<<conf_path<<std::endl;
	return 2;
    }
    nlohmann::json json;
    file>>json;
    file.close();

    std::string topic1 = json["topic"][0];
    std::string topic2 = json["topic"][1];
    std::string filename1 = json["res_filenames"][0];
    std::string filename2 = json["res_filenames"][1];
    int m_count = json["m_count"];
    int prior1 = json["priority"][0];
    int prior2 = json["priority"][1];
    int cpu1 = json["cpu_index"][0];
    int cpu2 = json["cpu_index"][1];
    int min_size = json["min_msg_size"];
    int max_size = json["max_msg_size"];
    int step = json["step"];
    int before_step = json["msgs_before_step"];
    int interval = json["interval"];
    int topic_prior = json["topic_priority"];

    try{

        if(type == "publisher"){
            std::cout<<"Publisher"<<std::endl;
            Publisher pub(address, topic1, m_count, prior1, cpu1,  min_size, max_size, step,
                    interval, before_step, filename1, topic_prior);
            pub.StartTest();
            std::cout<<"End Publisher"<<std::endl;
        }
        else if(type == "subscriber"){
            std::cout<<"Subscriber"<<std::endl;
            Subscriber<Message> sub(address, topic1, m_count, prior2, cpu2, filename2, topic_prior);
            sub.StartTest();
            std::cout<<"End Subscriber"<<std::endl;
        }
        else if(type == "ping_pong"){
            bool isFirst=program.get<bool>("--first");
            
            std::cout<<"PingPong"<<std::endl;
            if(isFirst){
                    PingPong<Message> ping_pong = (interval == 0)?
                        PingPong<Message>(address, topic1, topic2, m_count, prior1, cpu1, filename1, topic_prior,
                                                interval, min_size, isFirst):
                        PingPong<Message>(address, topic1, topic2, m_count, prior1, cpu1, filename1, topic_prior,
                                                interval, min_size, max_size, step, before_step, isFirst);
                    ping_pong.StartTest();
            }else{
                    PingPong<Message> ping_pong = (interval == 0)?
                        PingPong<Message>(address, topic1, topic2, m_count, prior2, cpu2, filename2, topic_prior,
                                                interval, min_size, isFirst):
                        PingPong<Message>(address, topic1, topic2, m_count, prior2, cpu2, filename2, topic_prior,
                                                interval, min_size, max_size, step, before_step, isFirst);
                    ping_pong.StartTest();	
            }
            std::cout<<"End PingPong"<<std::endl;
        }else{
            std::cout<<"Wrong node type"<<std::endl;
            return 3;
        }

    } catch (std::exception& e){
        std::cout << e.what() << std::endl;
        return 4;
    }

    return 0;
}
