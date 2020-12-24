#include "cyber/cyber.h"
#include "testing/proto/data.pb.h"

#include"testing/pub_interface.hpp"
#include"testing/sub_interface.hpp"
#include"testing/ping_pong_interface.hpp"
#include<cstring>
#include<argparse/argparse.hpp>

#include<string>
#include<cstring>
#include<chrono>
#include<ctime>
#include<iostream>

using apollo::cyber::Node;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using testing::proto::Message;

class Publisher: public TestMiddlewarePub{
private:
    std::unique_ptr<Node> node;
    std::shared_ptr<Writer<Message>> writer;
public:
    Publisher(std::string& name,
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
    std::unique_ptr<Node> node = apollo::cyber::CreateNode(name);
    writer = node->CreateWriter<Message>(topic);
    }

    ~Publisher(){
    }

    unsigned long publish(short id, unsigned size) override{
        std::string str(size,'a');
        unsigned long time=std::chrono::duration_cast<std::chrono::
                    nanoseconds>(std::chrono::high_resolution_clock::
                    now().time_since_epoch()).count();
        auto msg = std::make_shared<Message>();
        msg->set_id(id);
        msg->set_time(time);
        msg->set_data(str);
        writer->Write(msg);

        time=std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count()-time;
        return time;
    }

};

void MessageCallbackSub(const std::shared_ptr<testing::proto::Message>& msg);

class Subscriber: public TestMiddlewareSub<Message>{
private:
    std::unique_ptr<Node> node;
    std::shared_ptr<Reader<Message>> reader;
    bool rec;
public:
    Subscriber(std::string& name,
            std::string &topic, 
            int msgCount, 
            int prior,
            int cpu_index, 
            std::string &filename, 
            int topic_priority
            ): TestMiddlewareSub<Message>(topic, msgCount, prior,
                        cpu_index, filename, topic_priority)
    {
        std::unique_ptr<Node> node = apollo::cyber::CreateNode(name);
        apollo::cyber::ReaderConfig reader_config;
        reader_config.channel_name = topic;
        reader_config.pending_queue_size = 512;
        reader = node->CreateReader<Message>(reader_config, MessageCallbackSub);
        rec = false;
    }

    ~Subscriber(){
    }
    
    short get_id(Message &msg) override{
        return msg.id();
    }
    
    unsigned long get_timestamp(Message &msg) override{
        return msg.time();
    }
    
    bool receive() override{
        if(rec){
            rec = false;
            return true;
        }
        return false;
    }

    friend void MessageCallbackSub(const std::shared_ptr<testing::proto::Message>& msg);
};

void MessageCallbackPingPong(const std::shared_ptr<testing::proto::Message>& msg);

class PingPong: public TestMiddlewarePingPong<Message>{
private:
    std::unique_ptr<Node> node;
    std::shared_ptr<Reader<Message>> reader;
    bool rec;
    std::shared_ptr<Writer<Message>> writer;
public:
    PingPong(std::string& name,
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
            ): TestMiddlewarePingPong<Message>(topic1, topic2, msgCount, prior, cpu_index, filename,
                topic_priority, interval, msg_size, isFirst)
    {
    std::unique_ptr<Node> node = apollo::cyber::CreateNode(name);
    if(isFirst){
        writer = node->CreateWriter<Message>(topic1);
        apollo::cyber::ReaderConfig reader_config;
        reader_config.channel_name = topic2;
        reader_config.pending_queue_size = 512;
        reader = node->CreateReader<Message>(reader_config, MessageCallbackPingPong);
    }else{
        writer = node->CreateWriter<Message>(topic2);
        apollo::cyber::ReaderConfig reader_config;
        reader_config.channel_name = topic1;
        reader_config.pending_queue_size = 512;
        reader = node->CreateReader<Message>(reader_config, MessageCallbackPingPong);
    }
    rec = false;
    }
    
    PingPong(std::string& name,
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
            ): TestMiddlewarePingPong<Message>(topic1, topic2, msgCount, prior, cpu_index, filename,
                topic_priority, interval, msgSizeMin, msgSizeMax, step, before_step, isFirst)
    {
    std::unique_ptr<Node> node = apollo::cyber::CreateNode(name);
    if(isFirst){
        writer = node->CreateWriter<Message>(topic1);
        apollo::cyber::ReaderConfig reader_config;
        reader_config.channel_name = topic2;
        reader_config.pending_queue_size = 512;
        reader = node->CreateReader<Message>(reader_config, MessageCallbackPingPong);
    }else{
        writer = node->CreateWriter<Message>(topic2);
        apollo::cyber::ReaderConfig reader_config;
        reader_config.channel_name = topic1;
        reader_config.pending_queue_size = 512;
        reader = node->CreateReader<Message>(reader_config, MessageCallbackPingPong);
    }
    rec = false;
    }

    ~PingPong(){
    }

    void publish(short id, unsigned size) override{

        //std::cout << "Publish " << id << "\n";
        unsigned long time=std::chrono::duration_cast<std::chrono::
                    nanoseconds>(std::chrono::high_resolution_clock::
                    now().time_since_epoch()).count();
        std::string str(size,'a');
        auto msg = std::make_shared<Message>();
        msg->set_id(id);
        msg->set_time(time);
        msg->set_data(str);
        time=std::chrono::duration_cast<std::chrono::
                    nanoseconds>(std::chrono::high_resolution_clock::
                    now().time_since_epoch()).count() - time;
        writer->Write(msg);
        _write_msg_time[id] = time;
    }

    
    short get_id(Message &msg) override{
        return msg.id();
    }
    
    unsigned long get_timestamp(Message &msg) override{
        return msg.time();
    }
    
    bool receive() override{
        if(rec){
            rec = false;
            return true;
        }
        return false;
    }

    friend void MessageCallbackPingPong(const std::shared_ptr<testing::proto::Message>& msg);
};

static Subscriber* p_sub = nullptr;
static PingPong* p_ping_pong = nullptr;

void MessageCallbackSub(const std::shared_ptr<testing::proto::Message>& msg){
    unsigned long int time=std::chrono::duration_cast<std::chrono::
        nanoseconds>(std::chrono::high_resolution_clock::
        now().time_since_epoch()).count();
    Message m;
    m.set_id(msg->id());
    m.set_time(msg->time());
    m.set_data("");
    std::string str(msg->data());
    time=std::chrono::duration_cast<std::chrono::
        nanoseconds>(std::chrono::high_resolution_clock::
        now().time_since_epoch()).count()-time;
    p_sub->write_received_msg(m, time);
    p_sub->rec = true;

}

void MessageCallbackPingPong(const std::shared_ptr<testing::proto::Message>& msg){
    Message m;
    unsigned long time=std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count();
    m.set_id(msg->id());
    m.set_time(msg->time());
    m.set_data("");
    std::string str(msg->data());
    time=std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count() - time;
    p_ping_pong->write_received_msg(m);
    p_ping_pong->_read_msg_time[m.id()] = time;
    p_ping_pong->rec = true;
    //std::cout << "Receive " << m.id() << "\n";

}


int main(int argc, char** argv){
    FLAGS_logbuflevel = google::GLOG_INFO;
    argparse::ArgumentParser program("PubSub");
    
    program.add_argument("-c","--config")
            .required()
            .help("-c/--config is a config for testing");
    program.add_argument("-t","--type")
            .required()
            .help("-t/--type is a type of the node: publisher, subscriber or ping_pong");
    program.add_argument("--first")
            .implicit_value(true)
            .default_value(false)
            .help("--first is a config for ping_pong test");
    

    try{
        program.parse_args(argc, argv);
    }catch(const std::runtime_error& err){
        std::cout << err.what() << std::endl;
        std::cout << program<<std::endl;
        return 1;
    }
    auto type=program.get<std::string>("-t");
    auto conf_path=program.get<std::string>("-c");
    
    std::ifstream file(conf_path);
    if(!file){
        std::cout<<"Can't open file "<<conf_path<<std::endl;
        return 2;
    }
    nlohmann::json json;
    file>>json;
    file.close();
    
    std::string topic1=json["topic"][0];
    std::string topic2=json["topic"][1];
    std::string filename1=json["res_filenames"][0];
    std::string filename2=json["res_filenames"][1];
    int m_count=json["m_count"];
    int prior1=json["priority"][0];
    int prior2=json["priority"][1];
    int cpu1=json["cpu_index"][0];
    int cpu2=json["cpu_index"][1];
    int min_size=json["min_msg_size"];
    int max_size=json["max_msg_size"];
    int step=json["step"];
    int before_step=json["msgs_before_step"];
    int interval=json["interval"];
    int topic_prior=json["topic_priority"];
    
    unsigned long count_name=std::chrono::duration_cast<std::chrono::
        nanoseconds>(std::chrono::high_resolution_clock::
        now().time_since_epoch()).count() ;
    apollo::cyber::Init(argv[0]);
    if(type == "publisher"){
        std::string name=std::string("/pub");
        name+=std::to_string(count_name);

        std::cout<<"Publisher"<<std::endl;
        Publisher pub(name, topic1, m_count, prior1, cpu1,  min_size, max_size, step,
                interval, before_step, filename1, topic_prior);
        pub.StartTest();
        std::cout<<"End Publisher"<<std::endl;
    }
    else if(type == "subscriber"){
        std::string name=std::string("/sub");
        name+=std::to_string(count_name);

        std::cout<<"Subscriber"<<std::endl;
        Subscriber sub(name, topic1, m_count, prior2, cpu2, filename2, topic_prior);
        p_sub = &sub;
        sub.StartTest();
        std::cout<<"End Subscriber"<<std::endl;
    }
    else if(type == "ping_pong"){
        bool isFirst=program.get<bool>("--first");
        
        std::string name=std::string("/ping_pong");
        name+=std::to_string(count_name);

        std::cout<<"PingPong"<<std::endl;
        if(isFirst){
            PingPong ping_pong = (interval == 0)?
                PingPong(name, topic1, topic2, m_count, prior1, cpu1, filename1, topic_prior,
                            interval, min_size, isFirst):
                PingPong(name, topic1, topic2, m_count, prior1, cpu1, filename1, topic_prior,
                            interval, min_size, max_size, step, before_step, isFirst);
            p_ping_pong = &ping_pong;
            ping_pong.StartTest();
        }else{
            PingPong ping_pong = (interval == 0)?
                PingPong(name, topic1, topic2, m_count, prior2, cpu2, filename2, topic_prior,
                            interval, min_size, isFirst):
                PingPong(name, topic1, topic2, m_count, prior2, cpu2, filename2, topic_prior,
                            interval, min_size, max_size, step, before_step, isFirst);
            p_ping_pong = &ping_pong;
            ping_pong.StartTest();    
        }
        std::cout<<"End PingPong"<<std::endl;
    }else{
        std::cout<<"Wrong node type"<<std::endl;
        return 3;
    }

    return 0;
}
