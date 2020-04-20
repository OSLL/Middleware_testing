#include"msg.hpp"
#include<cstring>
#include<cstdint>


Message::Message(int id,size_t size,char c){
	this->id=id;
	str=std::string(size,c);
}
	

void Message::get_bytes(std::string& bytes){
	int len=str.length()+1+sizeof(int32_t)+sizeof(uint64_t) ;
	char s[len];
	*((int32_t*)(s))=id;
	*((uint64_t*)(s+sizeof(int32_t)))=timestamp;
	memcpy((void*)(s+sizeof(int32_t)+sizeof(uint64_t)),str.c_str(),str.length()+1);
	bytes=std::string(s,len);
}

void Message::set_from_bytes(std::string& bytes){
	id=*((int32_t*)(bytes.c_str()));
	timestamp=*((uint64_t*)(bytes.c_str()+sizeof(int32_t)));
	str=std::string((char*)(bytes.c_str()+sizeof(int32_t)+sizeof(uint64_t)));
}
