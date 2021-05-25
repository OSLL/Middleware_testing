#include<iceoryx_posh/popo/publisher.hpp>
#include<iceoryx_posh/popo/subscriber.hpp>
#include<iceoryx_posh/runtime/posh_runtime.hpp>

#include"../../interface/pub_interface.hpp"
#include"../../interface/sub_interface.hpp"
#include"../../interface/ping_pong_interface.hpp"
#include<cstring>
#include<argparse/argparse.hpp>

#include<string>
#include<cstring>
#include<chrono>
#include<ctime>
#include<iostream>

#define QUEUE_SIZE 256

struct Message{
	unsigned long int timestamp;
	short id;
	size_t len;
	char* str;
};

class Publisher: public TestMiddlewarePub{
private:
	std::string _name;
	iox::popo::Publisher* pub;
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
		iox::runtime::PoshRuntime::getInstance(name);
		char param[100];
		if(topic.length()<100) memcpy(param,topic.c_str(),topic.length()+1);
		else{
			memcpy(param,topic.c_str(),99);
			param[99]='\0';
		}
		pub=new iox::popo::Publisher({"Test","Iceoryx",param});
		pub->offer();
	}

	~Publisher(){
		pub->stopOffer();
		delete pub;
	}

	unsigned long publish(short id, unsigned size) override{
		std::string str(size,'a');
		unsigned long time=std::chrono::duration_cast<std::chrono::
                	nanoseconds>(std::chrono::high_resolution_clock::
                	now().time_since_epoch()).count();

		auto sample=static_cast<Message*>(pub->allocateChunk(size+sizeof(Message),true));
		sample->id=id;
		sample->timestamp=time;
		sample->len=size;
		memcpy(((void*)sample)+sizeof(Message),str.c_str(),sample->len);
		pub->sendChunk(sample);
		time=std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count()-time;
		return time;
	}

};


template<class MsgType>
class Subscriber: public TestMiddlewareSub<MsgType>{
private:
	std::string _name;
	iox::popo::Subscriber* sub;
public:
	Subscriber(std::string& name,
			std::string &topic, 
			int msgCount, 
			int prior,
			int cpu_index, 
			std::string &filename, 
			int topic_priority
			): TestMiddlewareSub<MsgType>(topic, msgCount, prior,
            			cpu_index, filename, topic_priority)
	{
		iox::runtime::PoshRuntime::getInstance(name);
		char param[100];
		if(topic.length()<100) memcpy(param,topic.c_str(),topic.length()+1);
		else{
			memcpy(param,topic.c_str(),99);
			param[99]='\0';
		}
		sub=new iox::popo::Subscriber({"Test", "Iceoryx",param});
		sub->subscribe(QUEUE_SIZE);
	}

	~Subscriber(){
		sub->unsubscribe();
		delete sub;
	}
	
	short get_id(MsgType &msg) override{
		return msg.id;
	}
	
	unsigned long get_timestamp(MsgType &msg) override{
		return msg.timestamp;
	}
	
	bool receive() override{
		const void* chunk=nullptr;
		unsigned long time=std::chrono::duration_cast<std::chrono::
			nanoseconds>(std::chrono::high_resolution_clock::
			now().time_since_epoch()).count();
		bool get=sub->getChunk(&chunk);
		if(get){
			auto sample=static_cast<const Message*>(chunk);
			Message msg;
			msg.timestamp=sample->timestamp;
			msg.id=sample->id;
			msg.len=sample->len;
			std::string str((char*)(((void*)sample)+sizeof(Message)),msg.len);
			sub->releaseChunk(chunk);

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
	std::string _name;
	iox::popo::Publisher* pub;
	iox::popo::Subscriber* sub;
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
		  int msgSizeMin,
                  int msgSizeMax,
                  int step,
                  int before_step, 
		  bool isFirst
			): TestMiddlewarePingPong<MsgType>(topic1, topic2, msgCount, prior, cpu_index, filename,
				topic_priority, interval, msgSizeMin, msgSizeMax, step, before_step, isFirst),
                        _name(name)
	{
            create();
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
		  int msg_size, 
		  bool isFirst
			): TestMiddlewarePingPong<MsgType>(topic1, topic2, msgCount, prior, cpu_index, filename,
				topic_priority, interval, msg_size, isFirst),
                        _name(name)
        {
            create();
        }

        void create(){
		iox::runtime::PoshRuntime::getInstance(_name);
                std::string topic1 = TestMiddlewarePingPong<Message>::_topic_name1;
                std::string topic2 = TestMiddlewarePingPong<Message>::_topic_name2;
		char param1[100];
		char param2[100];
		if(topic1.length()<100) memcpy(param1,topic1.c_str(),topic1.length()+1);
		else{
			memcpy(param1,topic1.c_str(),99);
			param1[99]='\0';
		}
		if(topic2.length()<100) memcpy(param2,topic2.c_str(),topic2.length()+1);
		else{
			memcpy(param2,topic2.c_str(),99);
			param2[99]='\0';
		}
		if(TestMiddlewarePingPong<Message>::_isFirst) pub=new iox::popo::Publisher({"Iceoryx",param1});
		else pub=new iox::popo::Publisher({"Iceoryx",param2});
		pub->offer();
		if(TestMiddlewarePingPong<Message>::_isFirst) sub=new iox::popo::Subscriber({"Iceoryx",param2});
		else sub=new iox::popo::Subscriber({"Iceoryx",param1});
		sub->subscribe(QUEUE_SIZE);
        }

	~PingPong(){
		pub->stopOffer();
		delete pub;
		sub->unsubscribe();
		delete sub;
	}

	void publish(short id, unsigned size) override{
		std::string str(size,'a');

		unsigned long time=std::chrono::duration_cast<std::chrono::
			nanoseconds>(std::chrono::high_resolution_clock::
			now().time_since_epoch()).count();
		auto sample=static_cast<Message*>(pub->allocateChunk(size+sizeof(Message),true));
		sample->id=id;
		sample->timestamp=std::chrono::duration_cast<std::chrono::
			nanoseconds>(std::chrono::high_resolution_clock::
			now().time_since_epoch()).count();
		sample->len=size;
		memcpy(((void*)sample)+sizeof(Message),str.c_str(),sample->len);
		pub->sendChunk(sample);
		time=std::chrono::duration_cast<std::chrono::
			nanoseconds>(std::chrono::high_resolution_clock::
			now().time_since_epoch()).count() - time;
                TestMiddlewarePingPong<MsgType>::_write_msg_time[id] = time;
	}

	
	short get_id(MsgType &msg) override{
		return msg.id;
	}
	
	unsigned long get_timestamp(MsgType &msg) override{
		return msg.timestamp;
	}
	
	bool receive() override{
		const void* chunk=nullptr;
		unsigned long time=std::chrono::duration_cast<std::chrono::
			nanoseconds>(std::chrono::high_resolution_clock::
			now().time_since_epoch()).count();
		bool get=sub->getChunk(&chunk);
		if(get){
			auto sample=static_cast<const Message*>(chunk);
			Message msg;
			msg.timestamp=sample->timestamp;
			msg.id=sample->id;
			msg.len=sample->len;
			std::string str((char*)(((void*)sample)+sizeof(Message)),msg.len);
			sub->releaseChunk(chunk);

			time=std::chrono::duration_cast<std::chrono::
				nanoseconds>(std::chrono::high_resolution_clock::
				now().time_since_epoch()).count()-time;
			//free(msg.str);
			//std::cout<<"Received: id - "<<msg.id<<" time - "<<msg.timestamp<<std::endl;
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
		Subscriber<Message> sub(name, topic1, m_count, prior2, cpu2, filename2, topic_prior);
		sub.StartTest();
		std::cout<<"End Subscriber"<<std::endl;
	}
	else if(type == "ping_pong"){
		bool isFirst=program.get<bool>("--first");
		
		std::string name=std::string("/ping_pong");
		name+=std::to_string(count_name);

		std::cout<<"PingPong"<<std::endl;
		if(isFirst){
			PingPong<Message> ping_pong = (interval == 0)? 
                            PingPong<Message>(name, topic1, topic2, m_count, prior1, cpu1, filename1, topic_prior,
						interval, min_size, isFirst):
                            PingPong<Message>(name, topic1, topic2, m_count, prior1, cpu1, filename1, topic_prior,
                                                interval, min_size, max_size, step, before_step, isFirst);

			ping_pong.StartTest();
		}else{
			PingPong<Message> ping_pong = (interval == 0)? 
                            PingPong<Message>(name, topic1, topic2, m_count, prior2, cpu2, filename2, topic_prior,
						interval, min_size, isFirst):
                            PingPong<Message>(name, topic1, topic2, m_count, prior2, cpu2, filename2, topic_prior,
                                                interval, min_size, max_size, step, before_step, isFirst);
			ping_pong.StartTest();	
		}
		std::cout<<"End PingPong"<<std::endl;
	}else{
		std::cout<<"Wrong node type"<<std::endl;
		return 3;
	}

	return 0;
}
