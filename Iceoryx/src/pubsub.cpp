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

struct Message{
	unsigned long int timestamp;
	short id;
	size_t len;
};

class Publisher: public TestMiddlewarePub{
private:
	std::string _name;
	iox::popo::Publisher* pub;
public:
	Publisher(std::string& topic,	
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
		iox::runtime::PoshRuntime::getInstance(std::string("/")+filename);
		char param[100];
		if(topic.length()<100) memcpy(param,topic.c_str(),topic.length()+1);
		else{
			memcpy(param,topic.c_str(),99);
			param[99]='\0';
		}
		pub=new iox::popo::Publisher({"Iceoryx",param});
		pub->offer();
	}

	~Publisher(){
		pub->stopOffer();
		delete pub;
	}

	unsigned long publish(short id, unsigned size) override{
		unsigned long time=std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count();

		auto sample=static_cast<Message*>(pub->allocateChunk(size));
		sample->id=id;
		sample->timestamp=time;
		sample->len=size-sizeof(Message);
		memset(sample+sizeof(Message),'a',sample->len);

		time=std::chrono::duration_cast<std::chrono::
                nanoseconds>(std::chrono::high_resolution_clock::
                now().time_since_epoch()).count()-time;

		pub->sendChunk(sample);
		return time;
	}

};


template<class MsgType>
class Subscriber: public TestMiddlewareSub<MsgType>{
private:
	std::string _name;
	iox::popo::Subscriber* sub;
public:
	Subscriber(std::string &topic, 
			int msgCount, 
			int prior,
			int cpu_index, 
			std::string &filename, 
			int topic_priority
			): TestMiddlewareSub<MsgType>(topic, msgCount, prior,
            			cpu_index, filename, topic_priority)
	{
		iox::runtime::PoshRuntime::getInstance(std::string("/")+filename);
		char param[100];
		if(topic.length()<100) memcpy(param,topic.c_str(),topic.length()+1);
		else{
			memcpy(param,topic.c_str(),99);
			param[99]='\0';
		}
		sub=new iox::popo::Subscriber({"Iceoryx",param});
		sub->subscribe(10);
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
		bool get=sub->getChunk(&chunk);
		if(get){
			auto sample=static_cast<const Message*>(chunk);
			unsigned long time=std::chrono::duration_cast<std::chrono::
				nanoseconds>(std::chrono::high_resolution_clock::
				now().time_since_epoch()).count();
			Message msg;
			msg.timestamp=sample->timestamp;
			msg.id=sample->id;
			msg.len=sample->len;
			std::string str((char*)sample+sizeof(Message),msg.len);
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
	PingPong(std::string& topic1,
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
		iox::runtime::PoshRuntime::getInstance(std::string("/")+filename);
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
		if(isFirst) pub=new iox::popo::Publisher({"Iceoryx",param1});
		else pub=new iox::popo::Publisher({"Iceoryx",param2});
		pub->offer();
		if(isFirst) sub=new iox::popo::Subscriber({"Iceoryx",param2});
		else sub=new iox::popo::Subscriber({"Iceoryx",param1});
		sub->subscribe(10);
	}

	~PingPong(){
		pub->stopOffer();
		delete pub;
		sub->unsubscribe();
		delete sub;
	}

	void publish(short id, unsigned size) override{

		auto sample=static_cast<Message*>(pub->allocateChunk(size));
		sample->id=id;
		sample->timestamp=std::chrono::duration_cast<std::chrono::
				nanoseconds>(std::chrono::high_resolution_clock::
				now().time_since_epoch()).count();
		sample->len=size-sizeof(Message);
		memset(sample+sizeof(Message),'a',sample->len);

		pub->sendChunk(sample);
		return ;
	}
	
	short get_id(MsgType &msg) override{
		return msg.id;
	}
	
	unsigned long get_timestamp(MsgType &msg) override{
		return msg.timestamp;
	}
	
	bool receive() override{
		const void* chunk=nullptr;
		bool get=sub->getChunk(&chunk);
		if(get){
			auto sample=static_cast<const Message*>(chunk);
			Message msg;
			msg.timestamp=sample->timestamp;
			msg.id=sample->id;
			msg.len=sample->len;
			std::string str((char*)sample+sizeof(Message),msg.len);
			sub->releaseChunk(chunk);

			TestMiddlewarePingPong<MsgType>::write_received_msg(msg);

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
			.help("--first is a config for ping_pong test");
	

	try{
		program.parse_args(argc, argv);
	}catch(const std::runtime_error& err){
		std::cout << err.what() << std::endl;
		std::cout << program<<std::endl;
		return 1;
	}
	auto type=program.get<std::string>("-t");
	if(!type.compare("publisher")){
		auto conf_path=program.get<std::string>("-c");
		std::ifstream file(conf_path);
		if(!file){
			std::cout<<"Can't open file "<<conf_path<<std::endl;
			return 2;
		}
		nlohmann::json json;
		file>>json;
		file.close();

		std::string topic=json["topic"];
		std::string filename=json["res_filenames"][0];
		int m_count=json["m_count"];
		int min_size=json["min_msg_size"];
		int max_size=json["max_msg_size"];
		int step=json["step"];
		int before_step=json["msgs_before_step"];
		int prior=json["priority"][0];
		int cpu=json["cpu_index"][0];
		int interval=json["interval"];
		int topic_prior=json["topic_priority"];

		std::string name=std::string("/pub");
		std::cout<<"Publisher"<<std::endl;
		Publisher pub(topic, m_count, prior, cpu,  min_size, max_size, step,
				interval, before_step, filename, topic_prior);
		pub.StartTest();
		std::cout<<"End Publisher"<<std::endl;
	}
	if(!type.compare("subscriber")){
		auto conf_path=program.get<std::string>("-c");
		std::ifstream file(conf_path);
		if(!file){
			std::cout<<"Can't open file "<<conf_path<<std::endl;
			return 2;
		}
		nlohmann::json json;
		file>>json;
		file.close();

		std::string topic=json["topic"];
		std::string filename=json["res_filenames"][1];
		int m_count=json["m_count"];
		int prior=json["priority"][1];
		int cpu=json["cpu_index"][1];
		int topic_prior=json["topic_priority"];

		std::string name=std::string("/sub");
		std::cout<<"Subscriber"<<std::endl;
		Subscriber<Message> sub(topic, m_count, prior, cpu, filename, topic_prior);
		sub.StartTest();
		std::cout<<"End Subscriber"<<std::endl;
	}
	if(!type.compare("ping_pong")){
		nlohmann::json json_pp;
		if(auto conf_pp=program.present("--first")){
			std::ifstream file=std::ifstream(*conf_pp);
			if(!file){
				std::cout<<"Can't open file "<<argv[4]<<std::endl;
				return 2;
			}
			file>>json_pp;
			file.close();
		}else{
			std::cout<<"No config for ping_pong test"<<std::endl;
			std::cout<<program<<std::endl;
			return 3;
		}
		bool isFirst=json_pp["isPingPong"];
		int i;
		if(isFirst) i=0;
		else i=1;
		std::string topic1=json_pp["topic"][i];
		std::string topic2=json_pp["topic"][i];
		std::string filename=json_pp["res_filenames"][i];
		int m_count=json_pp["m_count"];
		int min_size=json_pp["min_msg_size"];
		int prior=json_pp["priority"][i];
		int cpu=json_pp["cpu_index"][i];
		int interval=json_pp["interval"];
		int topic_prior=json_pp["topic_priority"];

		std::cout<<"PingPong"<<std::endl;
		PingPong<Message> ping_pong(topic1, topic2, m_count, prior, cpu, filename, topic_prior,
						interval, min_size, isFirst);
		ping_pong.StartTest();
		std::cout<<"End PingPong"<<std::endl;
	}

	return 0;
}
