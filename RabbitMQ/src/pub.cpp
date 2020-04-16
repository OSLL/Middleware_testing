#include<iostream>
#include<vector>
#include<chrono>
#include<thread>
#include<SimpleAmqpClient/SimpleAmqpClient.h>
#include"../../interface/pub_interface.hpp"
#include"msg.hpp"


namespace RabbitmqTest{
	class Publisher: public TestMiddlewarePub{
	private:
		AmqpClient::Channel::ptr_t connection;
		std::string _exchange;
		std::string _routing_key;
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
				const std::string& exchange="amq.direct",
				const std::string& routing_key="test"): 
			TestMiddlewarePub(topic,msgCount,prior,cpu_index,min_msg_size,max_msg_size,
					step,interval,msgs_before_step,filename,topic_priority),
			_exchange(exchange),
			_routing_key(routing_key)
		{
			connection=AmqpClient::Channel::Create(host,port,username,password,vhost,max_frame);
		}
		~Publisher(){};

		unsigned long int publish(short id, unsigned size) override{
			std::string msg;
			Message message(id,size,'a');
			unsigned long proc_time= std::chrono::duration_cast<std::chrono::
                		nanoseconds>(std::chrono::high_resolution_clock::
                		now().time_since_epoch()).count();
			message.timestamp=proc_time;
			message.get_bytes(msg);

			connection->BasicPublish(_exchange,
					_routing_key,
					AmqpClient::BasicMessage::Create(msg));
			return proc_time;
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
	RabbitmqTest::Publisher pub(topic,m_count,prior,cpu,min_size,
			max_size,step,interval,before_step,filename,topic_prior);
	pub.StartTest();
	return 0;
}
