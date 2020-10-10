#include"../../interface/pub_interface.hpp"
#include"../../interface/sub_interface.hpp"
#include"../../interface/ping_pong_interface.hpp"
#include"pub.h"
#include<cstring>
#include<argparse/argparse.hpp>

#include<string>
#include<cstring>
#include<chrono>
#include<ctime>
#include<iostream>

class Publisher: public TestMiddlewarePub{
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
            configure_server(topic, interval);
	}

	~Publisher(){
            stop_server();
	}

	unsigned long publish(short id, unsigned size) override{
		unsigned long time=std::chrono::duration_cast<std::chrono::
                	nanoseconds>(std::chrono::high_resolution_clock::
                	now().time_since_epoch()).count();
                setData(id, size);
		time=std::chrono::duration_cast<std::chrono::
                	nanoseconds>(std::chrono::high_resolution_clock::
                	now().time_since_epoch()).count()-time;
                return time;
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
	
	if(type == "publisher"){
		std::cout<<"Publisher"<<std::endl;
		Publisher pub(topic1, m_count, prior1, cpu1,  min_size, max_size, step,
				interval, before_step, filename1, topic_prior);
		pub.StartTest();
		std::cout<<"End Publisher"<<std::endl;
	}
	else if(type == "subscriber"){
		std::cout<<"Subscriber"<<std::endl;
	//	Subscriber<Message> sub(topic1, m_count, prior2, cpu2, filename2, topic_prior);
	//	sub.StartTest();
		std::cout<<"End Subscriber"<<std::endl;
	}
	else if(type == "ping_pong"){
		bool isFirst=program.get<bool>("--first");
		
		std::cout<<"PingPong"<<std::endl;
		if(isFirst){
		//	PingPong<Message> ping_pong(topic1, topic2, m_count, prior1, cpu1, filename1, topic_prior,
		//				interval, min_size, isFirst);
		//	ping_pong.StartTest();
		}else{
		//	PingPong<Message> ping_pong(topic1, topic2, m_count, prior2, cpu2, filename2, topic_prior,
		//				interval, min_size, isFirst);
		//	ping_pong.StartTest();	
		}
		std::cout<<"End PingPong"<<std::endl;
	}else{
		std::cout<<"Wrong node type"<<std::endl;
		return 3;
	}

	return 0;
}
