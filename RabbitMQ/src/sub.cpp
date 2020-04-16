#include<iostream>
#include<SimpleAmqpClient/SimpleAmqpClient.h>
#include"../../interface/sub_interface.hpp"
#include"msg.hpp"

namespace RabbitmqTest{
	template<class MsgType>
	class Subscriber:public TestMiddlewareSub<MsgType>{
	private:
		AmqpClient::Channel::ptr_t connection;
		std::string _exchange;
		std::string _routing_key;
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
				const std::string& exchange="amq.direct",
				const std::string& routing_key="test"): 
				TestMiddlewareSub<MsgType>(topic,msgCount,
						prior,cpu_index,filename,topic_priority),
				_exchange(exchange),
				_routing_key(routing_key)
		
		{
			connection=AmqpClient::Channel::Create(host,port,username,password,vhost,max_frame);
			connection->DeclareQueue("");
			connection->BindQueue("",exchange,routing_key);
		}
		~Subscriber(){
			connection->UnbindQueue("",_exchange,_routing_key);
			connection->DeleteQueue("");
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
			unsigned long int time = std::chrono::duration_cast<std::chrono::
                			nanoseconds>(std::chrono::high_resolution_clock::
                			now().time_since_epoch()).count();
			std::string str=enve->Message()->Body();
			Message msg;
			msg.set_from_bytes(str);
			TestMiddlewareSub<MsgType>::write_received_msg(msg,time);}
			return get;
		}
	};
}



int main(int argc,char** argv){
	if(argc<2){
		std::cout<<"No file name. Using: pub <filename>"<<std::endl;
		return 1;
	}
	std::ifstream file(argv[1]);
	if(!file){
		std::cout<<"Can't open file"<<std::endl;
		return 1;
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
	RabbitmqTest::Subscriber<Message> sub(topic,m_count,prior,cpu,filename,topic_prior);
	if(sub.StartTest()){
		std::cout<<"Error"<<std::endl;
	}
	return 0;
}
