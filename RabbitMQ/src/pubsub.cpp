#include<iostream>
#include<vector>
#include<chrono>
#include<thread>
#include<SimpleAmqpClient/SimpleAmqpClient.h>
#include"../../interface/pub_interface.hpp"
#include"../../interface/sub_interface.hpp"
#include"../../interface/ping_pong_interface.hpp"
#include<argparse/argparse.hpp>
#include"msg.hpp"



namespace RabbitmqTest{
	class Publisher: public TestMiddlewarePub{
	private:
		AmqpClient::Channel::ptr_t connection;
		std::string _exchange;
	public:
		Publisher(std::string &topic,
				int msgCount,
				int prior,
				int cpu_index,
				int min_msg_size,
				int max_msg_size, 
				int step, int interval, 
				int msgs_before_step,
				std::string &filename, 
	    			int topic_priority,
				const std::string& host="localhost", 
				int port=5672, const std::string& username="guest", 
				const std::string& password="guest", 
				const std::string& vhost="/", 
				int max_frame=131072,
				const std::string& exchange="amq.direct"): 
			TestMiddlewarePub(topic,msgCount,prior,cpu_index,min_msg_size,max_msg_size,
					step,interval,msgs_before_step,filename,topic_priority),
			_exchange(exchange)
		{
			connection=AmqpClient::Channel::Create(host,port,username,password,vhost,max_frame);
		}
		~Publisher(){};

		unsigned long int publish(short id, unsigned size) override{
			unsigned long proc_time= std::chrono::duration_cast<std::chrono::
                		nanoseconds>(std::chrono::high_resolution_clock::
                		now().time_since_epoch()).count();
			std::string msg;
			Message message(id,size,'a');
			message.timestamp=proc_time;
			message.get_bytes(msg);

			connection->BasicPublish(_exchange,
					_topic_name,
					AmqpClient::BasicMessage::Create(msg));
			proc_time= std::chrono::duration_cast<std::chrono::
                		nanoseconds>(std::chrono::high_resolution_clock::
                		now().time_since_epoch()).count() - proc_time;
			return proc_time;
		}

	};
	
	template<class MsgType>
	class Subscriber:public TestMiddlewareSub<MsgType>{
	private:
		AmqpClient::Channel::ptr_t connection;
		std::string _exchange;
	public:
		Subscriber( std::string &topic, 
				int msgCount, 
				int prior,
				int cpu_index, 
				std::string &filename, 
				int topic_priority,
				const std::string& host="localhost", 
				int port=5672, const std::string& username="guest", 
				const std::string& password="guest", 
				const std::string& vhost="/", 
				int max_frame=131072,
				const std::string& exchange="amq.direct"): 
				TestMiddlewareSub<MsgType>(topic,msgCount,
						prior,cpu_index,filename,topic_priority),
				_exchange(exchange)
		{
			connection=AmqpClient::Channel::Create(host,port,username,password,vhost,max_frame);
			connection->DeclareQueue("");
			connection->BindQueue("",exchange,TestMiddlewareSub<MsgType>::_topic_name);
		}
		~Subscriber(){
			connection->UnbindQueue("",_exchange,TestMiddlewareSub<MsgType>::_topic_name);
			connection->DeleteQueue("");
		}
		short get_id(MsgType &msg) override{
			return msg.id;
		}

		unsigned long get_timestamp(MsgType &msg) override{
			return msg.timestamp;
		}
		bool receive() override{
			unsigned long int time = std::chrono::duration_cast<std::chrono::
                			nanoseconds>(std::chrono::high_resolution_clock::
                			now().time_since_epoch()).count();
			AmqpClient::Envelope::ptr_t enve;
			bool get=connection->BasicGet(enve,"");
			if(get){
			std::string str=enve->Message()->Body();
			Message msg;
			msg.set_from_bytes(str);
			time = std::chrono::duration_cast<std::chrono::
                			nanoseconds>(std::chrono::high_resolution_clock::
                			now().time_since_epoch()).count() - time;
			TestMiddlewareSub<MsgType>::write_received_msg(msg,time);}
			return get;
		}
	};


	template<class MsgType>
	class PingPong: public TestMiddlewarePingPong<MsgType>{
	private:
		AmqpClient::Channel::ptr_t connection;
		std::string _exchange;
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
		  	bool isFirst,
			const std::string& host="localhost", 
			int port=5672, const std::string& username="guest", 
			const std::string& password="guest", 
			const std::string& vhost="/", 
			int max_frame=131072,
			const std::string& exchange="amq.direct"): 
			 TestMiddlewarePingPong<MsgType>(topic1, topic2, msgCount, prior, cpu_index, filename,
					topic_priority, interval, msg_size, isFirst),
			_exchange(exchange)
		{
			connection=AmqpClient::Channel::Create(host,port,username,password,vhost,max_frame);
			connection->DeclareQueue("");
			if(isFirst) connection->BindQueue("",_exchange,TestMiddlewarePingPong<MsgType>::_topic_name2);
			else connection->BindQueue("",_exchange,TestMiddlewarePingPong<MsgType>::_topic_name1);
		}
		
		~PingPong(){
			if(TestMiddlewarePingPong<MsgType>::_isFirst) connection->UnbindQueue("",_exchange,TestMiddlewarePingPong<MsgType>::_topic_name2);
			else connection->UnbindQueue("",_exchange,TestMiddlewarePingPong<MsgType>::_topic_name1);
			connection->DeleteQueue("");
		}
		
		void publish(short id, unsigned size) override{
			std::string msg;
			Message message(id,size,'a');
			message.timestamp=std::chrono::duration_cast<std::chrono::
				nanoseconds>(std::chrono::high_resolution_clock::
				now().time_since_epoch()).count();
			message.get_bytes(msg);

			if(TestMiddlewarePingPong<MsgType>::_isFirst) connection->BasicPublish(_exchange,
					TestMiddlewarePingPong<MsgType>::_topic_name1,
					AmqpClient::BasicMessage::Create(msg));
			else connection->BasicPublish(_exchange,
					TestMiddlewarePingPong<MsgType>::_topic_name2,
					AmqpClient::BasicMessage::Create(msg));
		}
		short get_id(MsgType &msg) override{
			return msg.id;
		}

		unsigned long get_timestamp(MsgType &msg) override{
			return msg.timestamp;
		}
		bool receive() override{
			AmqpClient::Envelope::ptr_t enve;
			bool get=connection->BasicGet(enve,"");
			if(get){
			std::string str=enve->Message()->Body();
			Message msg;
			msg.set_from_bytes(str);
			TestMiddlewarePingPong<MsgType>::write_received_msg(msg);}
			return get;
		}
	};
}

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
	program.add_argument("--connection")
			.required()
			.help("--connection is a config for connection");
	

	try{
		program.parse_args(argc, argv);
	}catch(const std::runtime_error& err){
		std::cout << err.what() << std::endl;
		std::cout << program<<std::endl;
		return 1;
	}
	auto conf_conn=program.get<std::string>("--connection");
	nlohmann::json json;
	std::ifstream file(conf_conn);
	if(!file){
		std::cout<<"Can't open file "<<conf_conn<<std::endl;
		return 2;
	}
	file>>json;
	std::string host=json["host"];
	int port=json["port"];
	std::string username=json["username"];
	std::string password=json["password"];
	std::string vhost=json["vhost"];
	int max_frame=json["max_frame"];
	std::string exchange=json["exchange"];
	file.close();
	auto type=program.get<std::string>("-t");
	if(!type.compare("publisher")){
		auto conf_path=program.get<std::string>("-c");
		file=std::ifstream(conf_path);
		if(!file){
			std::cout<<"Can't open file "<<conf_path<<std::endl;
			return 2;
		}
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
		RabbitmqTest::Publisher pub(topic, m_count, prior, cpu,  min_size, max_size, step,
				interval, before_step, filename, topic_prior,
				host, port, username, password, vhost, max_frame, exchange);
		pub.StartTest();
		std::cout<<"End Publisher"<<std::endl;
	}
	if(!type.compare("subscriber")){
		auto conf_path=program.get<std::string>("-c");
		file=std::ifstream(conf_path);
		if(!file){
			std::cout<<"Can't open file "<<conf_path<<std::endl;
			return 2;
		}
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
		RabbitmqTest::Subscriber<Message> sub(topic, m_count, prior, cpu, filename, topic_prior,
						host, port, username, password, vhost, max_frame, exchange);
		sub.StartTest();
		std::cout<<"End Subscriber"<<std::endl;
	}
	if(!type.compare("ping_pong")){
		nlohmann::json json_pp;
		if(auto conf_pp=program.present("--first")){
			file=std::ifstream(*conf_pp);
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
		RabbitmqTest::PingPong<Message> ping_pong(topic1, topic2, m_count, prior, cpu, filename, topic_prior,
						interval, min_size, isFirst,
						host, port, username, password, vhost, max_frame, exchange);
		ping_pong.StartTest();
		std::cout<<"End PingPong"<<std::endl;
	}

	return 0;
}
