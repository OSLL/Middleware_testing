
#include<SimpleAmqpClient/SimpleAmqpClient.h>

class MyMsg: public AmqpClient::BasicMessage{
public:
	short id;
};
