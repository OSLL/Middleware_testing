#include<iostream>
#include<chrono>
#include<thread>
#include<algorithm>
#include<gen/Types_DCPS.hpp>


int main(int argc, char** argv){
	dds::domain::DomainParticipant dom_par(0);
	dds::topic::Topic<os_sub_pub::StringMessage> topic(dom_par,"Topic");
	dds::sub::Subscriber sub(dom_par);
	dds::sub::DataReader<os_sub_pub::StringMessage> reader(sub,topic);
	
	std::cout<<"Start Reader"<<std::endl;
	dds::core::cond::WaitSet ws;
	dds::sub::cond::ReadCondition rc(reader,dds::sub::status::DataState::new_data());
	ws+=rc;

	while(true){
		ws.wait();
		auto samples=reader.read();
		for(dds::sub::Sample<os_sub_pub::StringMessage> samp : samples){
			const os_sub_pub::StringMessage& msg=samp.data();
			std::cout<<"Read: id: "<<msg.id()<<" message: "<<msg.msg()<<std::endl;
		}
	}

	return 0;
}
