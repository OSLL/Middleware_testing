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
}



int main(int argc,char** argv){
	if(argc<3){
		std::cout<<"No file name. Using: pub <config_for_connection> <config_for_test>"<<std::endl;
		return 1;
	}
	std::ifstream file(argv[2]);
	if(!file){
		std::cout<<"Can't open file "<<argv[2]<<std::endl;
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

	file.open(argv[1]);
	if(!file){
		std::cout<<"Can't open file "<<argv[1]<<std::endl;
		return 1;
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

	RabbitmqTest::Subscriber<Message> sub(topic,m_count,prior,cpu,filename,topic_prior,
						host,port,username,password,vhost,max_frame,exchange);
	if(sub.StartTest()){
		std::cout<<"Error"<<std::endl;
	}
	return 0;
}
