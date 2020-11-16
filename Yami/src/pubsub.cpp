#include <argparse/argparse.hpp>
#include <nlohmann/json.hpp>

#include"../../interface/pub_interface.hpp"
#include"../../interface/sub_interface.hpp"
#include"../../interface/ping_pong_interface.hpp"

#include<yami4-cpp/yami.h>

struct Message{
	short id;
	unsigned long int timestamp;
	std::string str;
};

//This function need for "publish_confirm"
//if not using send_one_way
void callback(yami::outgoing_message& om){
	yami::message_state state=om.get_state();
	if(state==yami::rejected){
		std::cout<<"Rejected"<<std::endl;
	}
}


class Publisher: public TestMiddlewarePub{
private:
	yami::agent agent;
	std::string _address;
        yami::value_publisher val;
public:
	Publisher(std::string& address,
		std::string &topic, 
		int msgCount, 
		int prior, 
		int cpu_index,
		int min_msg_size, 
		int max_msg_size, 
		int step, 
		int interval, 
		int msgs_before_step, 
		std::string &filename, 
		int topic_priority) :
    		TestMiddlewarePub(topic, msgCount, prior, cpu_index, min_msg_size, 
			max_msg_size, step, interval, msgs_before_step, 
			filename, topic_priority),
		_address(address)
	{
                agent.add_listener(_address);
                agent.register_value_publisher(topic, val);
	}
	~Publisher(){};

	unsigned long int publish(short id, unsigned int size) override{
		yami::parameters cont;
		std::string str(size,'a');
		unsigned long int time=std::chrono::duration_cast<std::chrono::
                	nanoseconds>(std::chrono::high_resolution_clock::
                	now().time_since_epoch()).count();
		cont.set_integer("id",id);
		cont.set_long_long("timestamp",time);
		cont.set_binary_shallow("str",str.c_str(),str.length());
		val.publish(cont);
		time=std::chrono::duration_cast<std::chrono::
                	nanoseconds>(std::chrono::high_resolution_clock::
                	now().time_since_epoch()).count()-time;
		return time;
	} 
};




template<class MsgType>
void update_sub(yami::incoming_message& message, void* sub);


template<class MsgType>
void update_ping_pong(yami::incoming_message& message, void* ping_pong);

template<class MsgType>
class Subscriber: public TestMiddlewareSub<MsgType>{
private:
	yami::agent agent;
        std::string _address;
	bool rec;
public:
	Subscriber(std::string& address,
		std::string &topic, 
		int msgCount, 
		int prior,
		int cpu_index, 
		std::string &filename, 
		int topic_priority) :
    		TestMiddlewareSub<MsgType>(topic, msgCount, prior, cpu_index, 
			filename, topic_priority),
		rec(false),
                _address(address)
	{
                unsigned long long int start, end;
                start = end = std::chrono::duration_cast<std::chrono::
                	nanoseconds>(std::chrono::high_resolution_clock::
                	now().time_since_epoch()).count();
                try{
		        agent.register_raw_object("handler",update_sub<MsgType>,this);
		        yami::parameters param;
		        param.set_string("destination_object","handler");
		        agent.send_one_way(_address,topic,"subscribe",param);
                }catch(yami::yami_runtime_error e){
                        end = std::chrono::duration_cast<std::chrono::
                	nanoseconds>(std::chrono::high_resolution_clock::
                	now().time_since_epoch()).count();
                        if(end - start > TIMEOUT) throw e;
                }
		std::cout<<"Subscribed"<<std::endl;

	}
	~Subscriber(){
            yami::parameters param;
	    param.set_string("destination_object","handler");
	    agent.send_one_way(_address,TestMiddlewareSub<MsgType>::_topic_name,"unsubscribe",param);
            
        };
	
	bool receive(){
		if(rec){
			rec=false;
			return true;
		}
		return false;
	}


   	short get_id(MsgType &msg){
		return msg.id;
	}
   	unsigned long get_timestamp(MsgType &msg){
		return msg.timestamp;
	}

	void set_rec(){rec=true;}
};


template<class MsgType>
class PingPong: public TestMiddlewarePingPong<MsgType>{
private:
	yami::agent agent;
        std::string _address1, _address2;
        yami::value_publisher val;
	bool rec;
public:
	PingPong(std::string& address,
		std::string &topic1, 
		std::string &topic2, 
		int msgCount, 
		int prior,
		int cpu_index, 
		std::string &filename, 
		int topic_priority, 
		int msInterval, 
		int msgSize, 
		bool isFirst) :
		TestMiddlewarePingPong<MsgType>(topic1, topic2, msgCount, prior, 
				cpu_index,filename, topic_priority, msInterval, 
				msgSize, isFirst),
		rec(false),
                _address1(address + '1'),
                _address2(address + '2')
	{
                if(isFirst) agent.add_listener(_address1);
                else agent.add_listener(_address2);
                unsigned long long int start, end;
                start = end = std::chrono::duration_cast<std::chrono::
                	nanoseconds>(std::chrono::high_resolution_clock::
                	now().time_since_epoch()).count();
                try{
		        agent.register_raw_object("handler",update_ping_pong<MsgType>,this);
		        yami::parameters param;
		        param.set_string("destination_object","handler");
	                if(isFirst){
                                agent.register_value_publisher(topic1, val);
                                agent.send_one_way(_address2,topic2,"subscribe",param);
                        }else{
                                agent.register_value_publisher(topic2, val);
                                agent.send_one_way(_address1,topic1,"subscribe",param);
                        }
                }catch(yami::yami_runtime_error e){
                        end = std::chrono::duration_cast<std::chrono::
                	nanoseconds>(std::chrono::high_resolution_clock::
                	now().time_since_epoch()).count();
                        if(end - start > TIMEOUT) throw e;
                }
		std::cout<<"Subscribed"<<std::endl;
	}
	
        PingPong(std::string& address,
		std::string &topic1, 
		std::string &topic2, 
		int msgCount, 
		int prior,
		int cpu_index, 
		std::string &filename, 
		int topic_priority, 
		int msInterval, 
		int msgSizeMin, 
		int msgSizeMax, 
                int step,
                int before_step,
		bool isFirst) :
		TestMiddlewarePingPong<MsgType>(topic1, topic2, msgCount, prior, 
				cpu_index,filename, topic_priority, msInterval, 
				msgSizeMin, msgSizeMax, step, before_step, isFirst),
		rec(false),
                _address1(address + '1'),
                _address2(address + '2')
	{
                if(isFirst) agent.add_listener(_address1);
                else agent.add_listener(_address2);
                unsigned long long int start, end;
                start = end = std::chrono::duration_cast<std::chrono::
                	nanoseconds>(std::chrono::high_resolution_clock::
                	now().time_since_epoch()).count();
                try{
		        agent.register_raw_object("handler",update_ping_pong<MsgType>,this);
		        yami::parameters param;
		        param.set_string("destination_object","handler");
	                if(isFirst){
                                agent.register_value_publisher(topic1, val);
                                agent.send_one_way(_address2,topic2,"subscribe",param);
                        }else{
                                agent.register_value_publisher(topic2, val);
                                agent.send_one_way(_address1,topic1,"subscribe",param);
                        }
                }catch(yami::yami_runtime_error e){
                        end = std::chrono::duration_cast<std::chrono::
                	nanoseconds>(std::chrono::high_resolution_clock::
                	now().time_since_epoch()).count();
                        if(end - start > TIMEOUT) throw e;
                }
		std::cout<<"Subscribed"<<std::endl;
	}
	
	~PingPong(){
                yami::parameters param;
	        param.set_string("destination_object","handler");
	        if(TestMiddlewarePingPong<MsgType>::_isFirst) agent.send_one_way(_address2,TestMiddlewarePingPong<MsgType>::_topic_name2,"unsubscribe",param);
                else agent.send_one_way(_address1,TestMiddlewarePingPong<MsgType>::_topic_name1,"unsubscribe",param);
        };
	
	bool receive(){
		if(rec){
			rec=false;
			return true;
		}
		return false;
	}


   	short get_id(MsgType &msg){
		return msg.id;
	}
   	unsigned long get_timestamp(MsgType &msg){
		return msg.timestamp;
	}

	void set_rec(){rec=true;}
	
        void publish(short id, unsigned int size) override{
		yami::parameters cont;
		std::string str(size,'a');
		unsigned long int time=std::chrono::duration_cast<std::chrono::
                	nanoseconds>(std::chrono::high_resolution_clock::
                	now().time_since_epoch()).count();
		cont.set_integer("id",id);
		cont.set_long_long("timestamp",time);
		cont.set_binary_shallow("str",str.c_str(),str.length());
                val.publish(cont);
		time=std::chrono::duration_cast<std::chrono::
                	nanoseconds>(std::chrono::high_resolution_clock::
                	now().time_since_epoch()).count() - time;
                TestMiddlewarePingPong<MsgType>::_write_msg_time[id] = time;
	} 

        void write_read_proc_time(int id, unsigned long time){
            TestMiddlewarePingPong<MsgType>::_read_msg_time[id] = time;
        }
	
};


template<class MsgType>
void update_sub(yami::incoming_message& message, void* sub){
	unsigned long int time=std::chrono::duration_cast<std::chrono::
               	nanoseconds>(std::chrono::high_resolution_clock::
               	now().time_since_epoch()).count();
	yami::parameters param=message.get_parameters();
	MsgType msg;
	msg.id=param.get_integer("id");
	msg.timestamp=param.get_long_long("timestamp");
        size_t len;
        const char* bin=(const char*)param.get_binary("str",len);
        std::string str(bin,len);
	time=std::chrono::duration_cast<std::chrono::
               	nanoseconds>(std::chrono::high_resolution_clock::
               	now().time_since_epoch()).count()-time;
	((Subscriber<MsgType>*)sub)->write_received_msg(msg,time);
	((Subscriber<MsgType>*)sub)->set_rec();
}

template<class MsgType>
void update_ping_pong(yami::incoming_message& message, void* ping_pong){
	unsigned long int time=std::chrono::duration_cast<std::chrono::
               	nanoseconds>(std::chrono::high_resolution_clock::
               	now().time_since_epoch()).count();
	yami::parameters param=message.get_parameters();
	MsgType msg;
	msg.id=param.get_integer("id");
	msg.timestamp=param.get_long_long("timestamp");
        size_t len;
        const char* bin=(const char*)param.get_binary("str",len);
        std::string str(bin,len);
	time=std::chrono::duration_cast<std::chrono::
               	nanoseconds>(std::chrono::high_resolution_clock::
               	now().time_since_epoch()).count()-time;
	((PingPong<MsgType>*)ping_pong)->write_received_msg(msg);
	((PingPong<MsgType>*)ping_pong)->write_read_proc_time(msg.id, time);
	((PingPong<MsgType>*)ping_pong)->set_rec();
}


int main(int argc, char** argv){
	argparse::ArgumentParser program("YamiPubSub");
	program.add_argument("-c", "--config")
			.required()
			.help("-c --conf is required argument with config path");
	program.add_argument("-t", "--type")
			.required()
			.help("-t --type is required argument with type of node");
	program.add_argument("-a","--adress")
			.required()
			.help("-a --address is a address for node");
	program.add_argument("--first")
			.implicit_value(true)
			.default_value(false)
			.help("--first is required argument if ping_pong type is specified with type of node");

	try{
		program.parse_args(argc, argv);
	}catch(const std::runtime_error& err){
		std::cout<<err.what()<<std::endl;
		std::cout<<program;
		return 1;
	}

	std::string conf=program.get<std::string>("-c");
	std::ifstream file(conf);
	if(!file){
		std::cout<<"Can't open file "<<conf<<std::endl;
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
	int min_msg_size=json["min_msg_size"];
	int max_msg_size=json["max_msg_size"];
	int step=json["step"];
	int before_step=json["msgs_before_step"];
	int interval=json["interval"];
	int topic_prior=json["topic_priority"];
	
	std::string type=program.get<std::string>("-t");
	
	std::string address=program.get<std::string>("-a");
	try{
		if(type=="publisher"){
			Publisher pub(address, topic1, m_count, prior1, cpu1, min_msg_size, 
				max_msg_size, step, interval, before_step, filename1, topic_prior);
			pub.StartTest();
		}
		if(type=="subscriber"){
			Subscriber<Message> sub(address, topic1, m_count, prior2, cpu2, filename2, topic_prior);
			sub.StartTest();
		}
		if(type=="ping_pong"){
			bool isFirst=program.get<bool>("--first");
			if(isFirst){
                                PingPong<Message> ping_pong = (interval == 0)? 
                                        PingPong<Message>(address, topic1, topic2, m_count, prior1, cpu1, filename1, topic_prior,
                                                        interval, min_msg_size, isFirst):
                                        PingPong<Message>(address, topic1, topic2, m_count, prior1, cpu1, filename1, topic_prior,
                                                        interval, min_msg_size, max_msg_size, step, before_step, isFirst);
                                ping_pong.StartTest();
			}else{
                                PingPong<Message> ping_pong = (interval == 0)? 
                                    PingPong<Message>(address, topic1, topic2, m_count, prior2, cpu2, filename2, topic_prior,
                                                        interval, min_msg_size, isFirst):
                                    PingPong<Message>(address, topic1, topic2, m_count, prior2, cpu2, filename2, topic_prior,
                                                        interval, min_msg_size, max_msg_size, step, before_step, isFirst);
                                ping_pong.StartTest();
			}
		}
	}catch(test_exception& e){
		std::cout<<e.what()<<std::endl;
		return -e.get_ret_code();
	}catch(std::exception& e){
		std::cout<<e.what()<<std::endl;
		return -MIDDLEWARE_ERROR;
	}
	return 0;
}
