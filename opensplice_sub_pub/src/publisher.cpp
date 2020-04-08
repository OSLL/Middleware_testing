#include<iostream>
#include<thread>
#include<chrono>
#include<vector>
#include<gen/Types_DCPS.hpp>


int main(int argc, char** argv){
	if(argc<2){
		std::cout<<"Missing id. Usage: pub <id>"<<std::endl;
		return 0;
	}
	int pub_id=atoi(argv[1]);
	dds::domain::DomainParticipant dom_par(0);
	dds::topic::Topic<os_sub_pub::StringMessage> topic(dom_par,"Topic");
	dds::pub::Publisher pub(dom_par);
	dds::pub::DataWriter<os_sub_pub::StringMessage> writer(pub,topic);
	
	std::cout<<"Start Writer"<<std::endl;

	os_sub_pub::StringMessage msg(pub_id,"");
	std::vector<std::string> vmsgs;
	vmsgs.push_back("This");
	vmsgs.push_back("is");
	vmsgs.push_back("a");
	vmsgs.push_back("Test!");
	vmsgs.push_back("Good");
	vmsgs.push_back("luck!");
	for(int i=0;i<vmsgs.size();i++){
		msg.msg(vmsgs[i]);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		writer.write(msg);
		std::cout<<"Writing: id: "<<msg.id()<<" message: "<<msg.msg()<<std::endl;
	}

	return 0;
}
