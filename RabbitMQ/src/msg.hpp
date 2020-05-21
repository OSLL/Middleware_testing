#ifndef MSGHPP
#define MSGHPP
#include<string>

class Message{
public:
	std::string str;
	int id;
	unsigned long int timestamp;
	
	Message(){}

	Message(int id,size_t size,char c);
	
	void get_bytes(std::string& bytes);

	void set_from_bytes(std::string& bytes);
};






#endif
